import math

import numpy as np
from numpy import sqrt

from splines.quaternion import Quaternion

import magpie.utils
from magpie.utils import skew_sym, vec_unit



def pose_vec_to_mtrx(vec):
    """
    Converts [translation + rotation] vector to a homogeneous transformation matrix.

    Parameters
    ----------
    vec: [6,] list
        Vector.\n
        [x, y, z, rX, rY, rZ]

    Returns
    -------
    matrix : [4,4] ndarray
        Homogeneous Transformation Matrix.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]
    """
    x, y, z, rx, ry, rz = vec
    theta = np.sqrt(np.square(rx) + np.square(ry) + np.square(rz))
    if theta == 0.0:
        theta = 1e-8

    kx = rx / theta
    ky = ry / theta
    kz = rz / theta
    cth = np.cos(theta)
    sth = np.sin(theta)
    vth = 1.0 - np.cos(theta)

    r11 = kx * kx * vth + cth
    r12 = kx * ky * vth - kz * sth
    r13 = kx * kz * vth + ky * sth
    r21 = kx * ky * vth + kz * sth
    r22 = ky * ky * vth + cth
    r23 = ky * kz * vth - kx * sth
    r31 = kx * kz * vth - ky * sth
    r32 = ky * kz * vth + kx * sth
    r33 = kz * kz * vth + cth

    matrix = np.eye(4)
    matrix[:3, -1] = np.array([x, y, z])
    matrix[:3, :3] = np.array([[r11, r12, r13], [r21, r22, r23], [r31, r32, r33]])
    return matrix


def repair_pose( dodgyHomogPose, getErr = 0 ):
    """ Attempt to construct a pose that passes the test """
    # NOTE: This function assumes a 4x4 numpy array
    bgnPose = np.array( dodgyHomogPose )
    rtnPose = bgnPose.copy()
    xBasis  = vec_unit( bgnPose[0:3,0] )
    yBasis  = vec_unit( bgnPose[0:3,1] )
    zBasis  = np.cross( xBasis, yBasis )
    yBasis  = np.cross( zBasis, xBasis )
    rtnPose[0:3,0] = xBasis
    rtnPose[0:3,1] = yBasis
    rtnPose[0:3,2] = zBasis
    if getErr:
        return rtnPose, np.linalg.norm( rtnPose - bgnPose )
    else:
        return rtnPose


def pose_mtrx_to_vec( matrix ):
    """
    Converts homogeneous transformation matrix to a [translation + rotation] vector.

    Parameters
    ----------
    matrix : [4,4] ndarray
        Homogeneous Transformation Matrix.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]

    Returns
    -------
    vector: [6,] list
        Vector.\n
        [x, y, z, rX, rY, rZ]
    """

    # Attempt to fix negligible errors, Otherwise complain
    # NOTE: Computing poses and copying poses from the robot printout often result in tiny tiny errors that make the checker mad
    if not is_rotation_mtrx( matrix[0:3, 0:3] ):
        nuMtx, err = repair_pose( matrix, getErr = 1 )
        if err > 0.005:
            raise ValueError( "pose_mtrx_to_vec: Homogeneous matrix contained invalid rotation!\n{matrix}" )
        else:
            matrix = nuMtx.copy()
    
    ##### Position: The Easy Part ################
    x = matrix[0, -1]
    y = matrix[1, -1]
    z = matrix[2, -1]

    ##### Orientation: The Tricky Part ################
    r11 = matrix[0, 0]
    r12 = matrix[0, 1]
    r13 = matrix[0, 2]
    r21 = matrix[1, 0]
    r22 = matrix[1, 1]
    r23 = matrix[1, 2]
    r31 = matrix[2, 0]
    r32 = matrix[2, 1]
    r33 = matrix[2, 2]

    # NOTE: THIS IS TO PREVENT FLIPPING FOR HAND-CODED AXIS-ALSIGNED POSES, WHICH SEEMS TO BE COMMON
    # NOTE: STOP REMOVING THIS PART, **I WILL FIND OUT**
    # https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToAngle/
    epsilon  = 1e-4 # margin to allow for rounding errors
    epsilon2 = 0.1 #- margin to distinguish between 0 and 180 degrees

    if ((np.abs(r12-r21) < epsilon) and (np.abs(r13-r31) < epsilon) and (np.abs(r23-r32)< epsilon)):
        # Singularity found, First check for identity matrix which must have +1 for all terms, in leading diagonal and zero in other terms
        if ((np.abs(r12+r21) < epsilon2) and (np.abs(r13+r31) < epsilon2) and (np.abs(r23+r32) < epsilon2) and (np.abs(r11+r22+r33-3.0) < epsilon2)):
			# this singularity is identity matrix so angle = 0
            theta = 0.0
            rv1   = 0.0
            rv2   = 0.0
            rv3   = 0.0
        else:
            theta = np.pi
            xx = (r11 + 1.0) / 2.0
            yy = (r22 + 1.0) / 2.0
            zz = (r33 + 1.0) / 2.0
            xy = (r12 + r21) / 4.0
            xz = (r13 + r31) / 4.0
            yz = (r23 + r32) / 4.0
            if ((xx > yy) and (xx > zz)): # m[0][0] is the largest diagonal term
                if (xx < epsilon):
                    kx = 0.0
                    ky = 0.7071
                    kz = 0.7071
                else:
                    kx = np.sqrt( xx )
                    ky = xy / kx
                    kz = xz / kx
            elif (yy > zz): # m[1][1] is the largest diagonal term
                if (yy < epsilon):
                    kx = 0.7071
                    ky = 0.0
                    kz = 0.7071
                else:
                    ky = np.sqrt( yy )
                    kx = xy / ky
                    kz = yz / ky
            else: # m[2][2] is the largest diagonal term so base result on this
                if (zz < epsilon):
                    kx = 0.7071
                    ky = 0.7071
                    kz = 0.0
                else:
                    kz = np.sqrt( zz )
                    kx = xz / kz
                    ky = yz / kz

		
    else:
        val = (r11 + r22 + r33 - 1) / 2.0
        while val < -1.0:
            val += 2.0
        while val > 1.0:
            val -= 2.0
        theta = np.arccos( val )

        if theta == 0.0:
            theta = 1e-8
        sth = np.sin(theta)
        kx = (r32 - r23) / (2 * sth)
        ky = (r13 - r31) / (2 * sth)
        kz = (r21 - r12) / (2 * sth)

    rv1 = theta * kx
    rv2 = theta * ky
    rv3 = theta * kz

    

    return [float(x), float(y), float(z), float(rv1), float(rv2), float(rv3)]


def origin_pose():
    """Return the homogeneous pose corresponding to the origin, with no rotation"""
    return np.eye(4)


def transform_points(points, pose):
    """
    Transform points by a transformation matrix.

    Parameters
    ----------
    points: [n,3] ndarray
        Points.
    transform: [4,4] ndarray
        Transformation matrix to be used.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]

    Returns
    -------
    transformed_points: [n,3] ndarray
        The transformed points.
    """
    points = np.hstack((points, np.ones((points.shape[0], 1))))
    points = points.dot(pose.T)[:, :3]
    return points


def translate_pose(pose, translation_vec, dir_pose="self"):
    """
    Alters a transformation matrix, translating it by some amount along \
    each of its axes. 

    Parameters
    ----------
    transform: [4,4] ndarray
        The transformation_matrix.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]
    x_offset: float (m)
        The amount to translate the transform along its x axis.
    y_offset: float (m)
        The amount to translate the transform along its y axis.
    z_offset: float (m)
        The amount to translate the transform along its z axis.

    Returns
    -------
    transform_translate: [4,4] ndarray
        The original transformation matrix translated by the desired amount along each axis.
    """

    [x, y, z] = translation_vec

    if is_pose_mtrx(dir_pose):
        dir_pose = combine_rot_and_trans_from_poses(rot_pose=dir_pose, trans_pose=pose)
    elif dir_pose == "self":  # Analogous to tool frame
        dir_pose = pose
    elif dir_pose == "origin":  # Analogous to base frame
        dir_pose = combine_rot_and_trans_from_poses(rot_pose=np.eye(4), trans_pose=pose)
    else:
        raise Exception('dir_pose must be "self", "origin", or a pose matrix')

    end_point = np.hstack((np.eye(3), np.ones((3, 1))))
    end_pose = dir_pose.dot(end_point.T)

    x_axis = end_pose[:3, 0] - dir_pose[:3, 3]
    y_axis = end_pose[:3, 1] - dir_pose[:3, 3]
    z_axis = end_pose[:3, 2] - dir_pose[:3, 3]

    translation = x * x_axis + y * y_axis + z * z_axis

    pose_translate = np.copy(pose)
    pose_translate[:3, 3] = pose_translate[:3, 3] + translation

    return pose_translate


def rotate_pose(pose, rotation_vec, dir_pose="self"):
    """
    Alters a transformation matrix, rotating it by some amount around \
    each of its axes. 
    Parameters
    ----------
    transform: [4,4] ndarray
        Transformation matrix.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]
    rx: float (rad)
        The amount to rotate the transform around its x axis.
    ry: float (rad)
        The amount to rotate the transform around its y axis.
    rz: float (rad)
        The amount to rotate the transform around its z axis.
    Returns
    -------
    transform_rotate: [4,4] ndarray
        The original transformation matrix rotated by the desired amount around each axis.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]
    """

    [rx, ry, rz] = rotation_vec
    rx_mat = np.array(
        ([1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)])
    )
    ry_mat = np.array(
        ([np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)])
    )
    rz_mat = np.array(
        ([np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1])
    )
    pose_rotate = np.copy(pose)

    if dir_pose == "self":
        pose_rotate[:3, :3] = pose_rotate[:3, :3].dot(rx_mat).dot(ry_mat).dot(rz_mat)
    elif dir_pose == "origin":
        pose_rotate[:3, :3] = rx_mat.dot(ry_mat).dot(rz_mat).dot(pose_rotate[:3, :3])

    return pose_rotate


def invert_pose(pose):
    """ 
    Return the transform T^{-1} that is the reverse of T. If points are transformed \
    by T, transforming the points then by T^{-1} would return them to their original positions. 

    Parameters
    ----------
    transform: [4,4] ndarray
        Transformation matrix.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]

    Returns
    -------
    inv_transform: [4,4] ndarray
        The inverse transformation matrix.\n
        [r11,r12,r13,tx]\n
        [r21,r22,r23,ty]\n
        [r31,r32,r33,tz]\n
        [ 0 , 0 , 0 , 1]
    """

    return np.linalg.inv(pose)


def is_pose_mtrx(pose):
    if type(pose) is not np.ndarray:
        return False
    if pose.shape == (4, 4):
        is_4x4 = True
    else:
        is_4x4 = False
    if is_rotation_mtrx(pose[0:3, 0:3]) and is_4x4:
        return True
    else:
        return False


def is_rotation_mtrx(R):
    # Checks if a matrix is a valid rotation matrix.
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    # print( f"Rt*R = \n{shouldBeIdentity}" )
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    e = 1e-5
    if n < e:
        return True
    else:
        # print( f"Difference from Identity: {n} > {e}" )
        return False


def rotation_mtrx_to_rpy(R):
    # Calculates rotation matrix to euler angles
    assert is_rotation_mtrx(R)

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])


def rpy_to_rotation_mtrx(theta):
    # Calculates Rotation Matrix given euler angles
    R_x = np.array(
        [
            [1, 0, 0],
            [0, math.cos(theta[0]), -math.sin(theta[0])],
            [0, math.sin(theta[0]), math.cos(theta[0])],
        ]
    )
    R_y = np.array(
        [
            [math.cos(theta[1]), 0, math.sin(theta[1])],
            [0, 1, 0],
            [-math.sin(theta[1]), 0, math.cos(theta[1])],
        ]
    )
    R_z = np.array(
        [
            [math.cos(theta[2]), -math.sin(theta[2]), 0],
            [math.sin(theta[2]), math.cos(theta[2]), 0],
            [0, 0, 1],
        ]
    )
    R = np.dot(R_z, np.dot(R_y, R_x))
    return R


def rotMatx_2_quat( r ):
    """ Get the unit quaternion associated with a rotation matrix """
    # r = np.around( r, 4 )
    # print( r )
    # print( 
    #     1.0 + r[0,0] - r[1,1] - r[2,2] + 0.0,
    #     1.0 - r[0,0] + r[1,1] - r[2,2] + 0.0,
    #     1.0 - r[0,0] - r[1,1] + r[2,2] + 0.0,
    #     1.0 + r[0,0] + r[1,1] + r[2,2] + 0.0
    # )
    sigDig = 6
    x = 0.5 * sqrt( np.round( 1.0 + r[0,0] - r[1,1] - r[2,2] + 0.0, sigDig ) )
    y = 0.5 * sqrt( np.round( 1.0 - r[0,0] + r[1,1] - r[2,2] + 0.0, sigDig ) )
    z = 0.5 * sqrt( np.round( 1.0 - r[0,0] - r[1,1] + r[2,2] + 0.0, sigDig ) )
    w = 0.5 * sqrt( np.round( 1.0 + r[0,0] + r[1,1] + r[2,2] + 0.0, sigDig ) )
    # print( w, [x,y,z] )
    return Quaternion( w, [x,y,z] ).normalized()


########## POSE ERROR ############################


def translation_diff( pose1, pose2 ):
    """ Return the linear separation between two poses """
    return np.linalg.norm( np.subtract( pose1[0:3,3], pose2[0:3,3] ) )


def orientation_error_vec( poseMeasure, poseDesired ):
    """ Return the difference in orientation between `poseMeasure` and `poseDesired` """
    # Implement the orientation error between quaternions from Siciliano et al. @ "3.7.3 Orientation Error"
    
    quatMeasure = rotMatx_2_quat( poseMeasure[0:3,0:3] )
    qM_1, qM_2, qM_3, qM_0 = quatMeasure.xyzw
    scalM = qM_0
    vectM = np.array([qM_1, qM_2, qM_3])
    
    quatDesired = rotMatx_2_quat( poseDesired[0:3,0:3] )
    qD_1, qD_2, qD_3, qD_0 = quatDesired.xyzw
    scalD = qD_0
    vectD = np.array([qD_1, qD_2, qD_3])
    
    # return scalM*vectD - scalD*vectM - skew_sym_cross( vectD ).dot( vectM )
    return scalM*vectD - scalD*vectM - skew_sym( vectD ).dot( vectM )


def orientation_error( poseMeasure, poseDesired ):
    """ Return the max absolute orientation difference about any axis """
    # Implement the orientation error between quaternions from Siciliano et al. @ "3.7.3 Orientation Error"
    return np.amax( np.absolute( orientation_error_vec( poseMeasure, poseDesired ) ) )


def pose_error( poseMeasure, poseDesired ):
    """ Return the translation and rotation error between the two poses """
    poseMeasure = np.around( poseMeasure.copy(), 4 )
    poseDesired = np.around( poseDesired.copy(), 4 )
    return np.array([
        translation_diff( poseMeasure, poseDesired ),
        orientation_error( poseMeasure, poseDesired ),
    ])


# def cartesian_mtrx_of_poses(
#     start_pose, x_range, y_range, z_range, frame="tool", output=None
# ):
#     matrix = np.zeros((len(x_range), len(y_range), len(z_range), 4, 4))

#     for i, x in enumerate(x_range):
#         for j, y in enumerate(y_range):
#             for k, z in enumerate(z_range):

#                 if frame == "base":
#                     matrix[i, j, k, :] = translate_pose(
#                         start_pose, x=x, y=y, z=z, frame="base"
#                     )
#                 if frame == "tool":
#                     matrix[i, j, k, :] = translate_pose(
#                         start_pose, x=x, y=y, z=z, frame="self"
#                     )

#     if output == "all":
#         shape = matrix.shape
#         view = self.PC_Viewer()
#         for i in range(shape[0]):
#             for j in range(shape[1]):
#                 for k in range(shape[2]):
#                     view.add_axis(matrix[i, j, k, :])
#         view.show()

#     return matrix


# def cylindircal_mtrx_of_poses(
#     start_pose,
#     r_range,
#     theta_range,
#     z_range,
#     frame="tool",
#     cylinder_axis="y",
#     output=None,
# ):
#     matrix = np.zeros((len(r_range), len(theta_range), len(z_range), 4, 4))

#     for i, r in enumerate(r_range):
#         for j, theta in enumerate(theta_range):
#             for k, z in enumerate(z_range):

#                 if frame == "tool":
#                     pose = translate_pose(start_pose, z=r_range[0], frame="self")
#                     if cylinder_axis == "y":
#                         pose = rotate_pose(pose, ry=math.radians(theta), frame="self")
#                         matrix[i, j, k, :] = translate_pose(
#                             pose, y=z, z=-r, frame="self"
#                         )
#                     elif cylinder_axis == "x":
#                         pose = rotate_pose(pose, rx=math.radians(theta), frame="self")
#                         matrix[i, j, k, :] = translate_pose(
#                             pose, x=z, z=-r, frame="self"
#                         )
#                     elif cylinder_axis == "z":
#                         pose = rotate_pose(pose, rz=math.radians(theta), frame="self")
#                         matrix[i, j, k, :] = translate_pose(
#                             pose, x=z, z=-r, frame="self"
#                         )

#     if output == "all":
#         shape = matrix.shape
#         view = self.PC_Viewer()
#         for i in range(shape[0]):
#             for j in range(shape[1]):
#                 for k in range(shape[2]):
#                     view.add_axis(matrix[i, j, k, :])

#         view.show()
#     return matrix


# def spherical_mtrx_of_poses(
#     start_pose, r_range, theta_range, phi_range, frame="tool", output=None
# ):
#     matrix = np.zeros((len(phi_range), len(theta_range), len(r_range), 4, 4))

#     for i, phi in enumerate(phi_range):
#         for j, theta in enumerate(theta_range):
#             for k, r in enumerate(r_range):

#                 if frame == "tool":
#                     pose = translate_pose(start_pose, z=r_range[0], frame="self")
#                     pose = rotate_pose(
#                         pose, ry=math.radians(phi), rx=math.radians(theta), frame="self"
#                     )
#                     matrix[i, j, k, :] = translate_pose(pose, z=-r, frame="self")

#     if output == "all":
#         shape = matrix.shape
#         view = self.PC_Viewer()
#         for i in range(shape[0]):
#             for j in range(shape[1]):
#                 for k in range(shape[2]):
#                     view.add_axis(matrix[i, j, k, :])

#         view.show()
#     return matrix


# def get_distance_between_poses(pose_1, pose_2):
#     """Return the Euclidian distance between the displacement vectors of the respective poses"""
#     point_a = pose_1[:3, 3]
#     point_b = pose_2[:3, 3]
#     return np.linalg.norm(point_a - point_b)


# def rotMatx_to_quat(R):
#     """Return the quaternion WXYZ that corresponds to the given 3x3 rotation matrix `R`"""
#     quatWXYZ = np.zeros(4)
#     e0Sqr = 0.25 * (1 + R[0, 0] + R[1, 1] + R[2, 2])
#     quatWXYZ[0] = math.sqrt(e0Sqr)
#     w = quatWXYZ[0]
#     if e0Sqr > 0:
#         quatWXYZ[1] = (1 / (4 * w)) * (R[2, 1] - R[1, 2])
#         quatWXYZ[2] = (1 / (4 * w)) * (R[0, 2] - R[2, 0])
#         quatWXYZ[3] = (1 / (4 * w)) * (R[1, 0] - R[0, 1])
#     else:
#         e1Sqr = -0.5 * (R[1, 1] + R[2, 2])
#         quatWXYZ[1] = math.sqrt(e1Sqr)
#         x = quatWXYZ[1]
#         if e1Sqr > 0:
#             quatWXYZ[2] = R[0, 1] / (2 * x)
#             quatWXYZ[3] = R[0, 2] / (2 * x)
#         else:
#             e2Sqr = 0.5 * (1 - R[2, 2])
#             quatWXYZ[2] = math.sqrt(e2Sqr)
#             y = quatWXYZ[2]
#             if e2Sqr > 0:
#                 quatWXYZ[3] = R[1, 2] / (2 * y)
#             else:
#                 quatWXYZ[3] = 1
#     return np.array(quatWXYZ)


# def sclr(quat):
#     """Retrieve the scalar component 'w' of the quaternion [ w x y z ]"""
#     return quat[0]


# def vctr(quat):
#     """Retrieve the vector component [ x y z ]' of the quaternion [ w x y z ]"""
#     return np.array([quat[1], quat[2], quat[3]])


# def skew_sym(vec):
#     """Return the skew symmetic matrix for the equivalent cross operation: [r_cross][v] = cross( r , v )"""
#     return np.array(
#         [[0.0, -vec[2], vec[1]], [vec[2], 0.0, -vec[0]], [-vec[1], vec[0], 0.0]]
#     )


# def orientation_error(quatMeasure, quatDesired):
#     """Implement the orientation error between quaternions from Siciliano et al."""
#     # NOTE: This function assumes that 'quatMeasure' and 'quatDesired' are properly-formed, unit quaternions [ w x y z ]
#     # NOTE: Returns an error vector with all 3 rotation components
#     #     print( "DEBUG, op1:" , quatMeasure )
#     #     print( "DEBUG, op2:" , quatDesired )
#     #     print( "DEBUG:" , sclr(quatMeasure) * vctr(quatDesired) )
#     #     print( "DEBUG:" , sclr(quatDesired) * vctr(quatMeasure) )
#     #     print( "DEBUG:" , skew_sym(vctr(quatDesired)).dot(vctr(quatMeasure)) )
#     return (
#         sclr(quatMeasure) * vctr(quatDesired)
#         - sclr(quatDesired) * vctr(quatMeasure)
#         - skew_sym(vctr(quatDesired)).dot(vctr(quatMeasure))
#     )


# def orient_err_R(Rmeasure, Rdesired):
#     """Implement the orientation error between quaternions from Siciliano et al."""
#     # NOTE: This function assumes that 'Rmeasure' and 'Rdesired' are properly-formed rotation matrices
#     # NOTE: Returns an error vector with all 3 rotation components
#     return orientation_error(rotMatx_to_quat(Rmeasure), rotMatx_to_quat(Rdesired))


# def pose_components(pose):
#     """Break the pose down into it's constituent parts and return as a dictionary"""
#     return {
#         "position": pose[0:3, 3],
#         "rotation": pose[0:3, 0:3],
#         "xBasis": utils.vec_unit(np.array(pose[0:3, 0])),
#         "yBasis": utils.vec_unit(np.array(pose[0:3, 1])),
#         "zBasis": utils.vec_unit(np.array(pose[0:3, 2])),
#     }


# def orient_error_between_poses(pose_1, pose_2, scalar=True):
#     """Error between pose orientations (R) from Siciliano et al."""
#     # NOTE: `scalar` flag returns the max error about any axis, otherwise return an error vector with all 3 rotation components
#     err = orient_err_R(
#         pose_components(pose_1)["rotation"], pose_components(pose_2)["rotation"]
#     )
#     if scalar:
#         return max(err)
#     else:
#         return err


# def combine_rot_and_trans_from_poses(rot_pose=None, trans_pose=None):
#     """Return a pose that is the position of `trans_pose` and the orientation of `rot_pose`"""
#     pose = np.ones((4, 4))
#     pose[:4, :3] = rot_pose[:4, :3]
#     pose[:4, 3] = trans_pose[:4, 3]
#     return pose


# def pose_from_three_poses(origin_pose, x_pose, y_pose):
#     """Return an origin pose from 3 taught points"""
#     pt1 = origin_pose[:3, 3]
#     pt2 = x_pose[:3, 3]
#     pt3 = y_pose[:3, 3]

#     rtn_pose = np.eye(4)

#     x_vec = (pt2 - pt1) / np.linalg.norm(pt2 - pt1)
#     y_vec = (pt3 - pt1) / np.linalg.norm(pt3 - pt1)

#     angle_error = (1.57 - np.arccos(np.dot(x_vec, y_vec))) / 2.0

#     z_vec = np.cross(x_vec, y_vec)
#     z_vec = z_vec / np.linalg.norm(z_vec)

#     y_vec = np.cross(z_vec, x_vec)
#     y_vec = y_vec / np.linalg.norm(y_vec)

#     rot = np.array((x_vec, y_vec, z_vec)).T

#     rtn_pose[:3, :3] = rot
#     rtn_pose[:3, 3] = pt1.T
#     rtn_pose = rotate_pose(rtn_pose, rz=-angle_error, frame="self")
#     return rtn_pose
