'''
@file pcd.py
@brief Utility functions to manipulate point cloud data in open3d
        and get PCA pose estimation given 3D point cloud
'''
import os, sys
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R
from open3d.web_visualizer import draw
import copy
import time

_VERBOSE = 0

def create_depth_mask_from_mask(mask, orig_depth):
    '''
    @param mask np.array of item mask
    @param orig_depth Open3D Image
    '''
    # depth_m_array = np.zeros_like(np.asarray(orig_depth))
    depth_m_array = np.asarray(orig_depth)
    depth_m_array[~mask] = 0
    depth_m = o3d.geometry.Image((depth_m_array).astype(np.float32))
    return depth_m

def crop_and_denoise_pcd(depth_m, orig_pcd, rsc, NB=50):
    '''
    @param depth_m Open3D Image, single channel depth image
    @param orig_pcd uncropped Open3D point cloud
    @param rsc RealSense object from realsense_wrapper.py
    '''
    ###
    # This is the magic line
    # find the depth mask from the OWL ViT bounding box
    # reassign the orig rgbdImage
    # then recreate the pcd
    ###
    orig_pcd.depth = depth_m
    # et voila
    if _VERBOSE:
        print( "About to `o3d.geometry.PointCloud.create_from_rgbd_image` ...", flush=True, file=sys.stderr )
    cpcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        orig_pcd,
        # rgbd_m_image,
        rsc.pinholeInstrinsics,
        project_valid_depth_only=True,
        extrinsic=rsc.extrinsics
    )

    if 0:

        # FIXME: POSSIBLE REMEDY FOR MULTIPROCESS PROBLEM, https://github.com/isl-org/Open3D/issues/4007

        # denoise pcd
        if _VERBOSE:
            print( f"About to `cpcd.remove_statistical_outlier` on {cpcd}\nRequired Neighbors: {NB} ...", flush=True, file=sys.stderr )

        # cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.001 )
        # cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.002 )
        # cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.005 )
        cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.01 )
        # cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.02 )
        # cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.05 )
        # cl, ind = cpcd.remove_statistical_outlier( nb_neighbors = NB, std_ratio = 0.10 )

        if _VERBOSE:
            print( "About to `cpcd.select_by_index` ...", flush=True, file=sys.stderr )
        inlier_cloud = cpcd.select_by_index( ind )
        # display_inlier_outlier(saved_pcd, ind)
        # displayWorld(inlier_cloud)
    else:
        inlier_cloud = cpcd

    return inlier_cloud

def get_minimum_width(pcd):
    mobb = pcd.get_minimal_oriented_bounding_box()
    obb  = pcd.get_oriented_bounding_box()
    mobb.extent, obb.extent
    width = np.min([*mobb.extent[:3], *obb.extent[:3]])
    return width

def retrieve_mask_from_image_crop(box, full_o3d_image):
    '''
    @param box 2d bounding box in form [x0, y0, x1, y1]
    @param full_o3d_image Open3D RGBD Image
    '''
    x_min = int(box[0])
    y_min = int(box[1])
    x_max = int(box[2])
    y_max = int(box[3])
    # y_min = int(box[0])
    # x_min = int(box[1])
    # y_max = int(box[2])
    # x_max = int(box[3])
    x_center = (x_min + x_max) / 2
    y_center = (y_min + y_max) / 2
    bbox = x_min, y_min, x_max, y_max

    # mask out bounding box in img
    depth_image = np.asarray(full_o3d_image.depth)
    # depth_values = depth_image[x_min:x_max, y_min:y_max]
    depth_values = depth_image[y_min:y_max, x_min:x_max]
    depth_o3d = o3d.geometry.Image((depth_values).astype(np.uint8))
    rgb_image    = np.asarray(full_o3d_image.color)
    # rgb_values = np.asarray(rgbdImage.color)[x_min:x_max, y_min:y_max]
    rgb_values = np.asarray(full_o3d_image.color)[y_min:y_max, x_min:x_max]
    rgb_o3d = o3d.geometry.Image((rgb_values).astype(np.uint8))

    cropped_o3d_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_o3d, depth_o3d)

    # Also, assuming x_min, x_max, y_min, y_max define your region of interest (ROI)

    # Create masks for the region of interest
    roi_mask_rgb = np.zeros_like(rgb_image, dtype=bool)
    # roi_mask_rgb[x_min:x_max, y_min:y_max, :] = True
    roi_mask_rgb[y_min:y_max, x_min:x_max, :] = True

    roi_mask_depth = np.zeros_like(depth_image, dtype=bool)
    # roi_mask_depth[x_min:x_max, y_min:y_max] = True
    roi_mask_depth[y_min:y_max, x_min:x_max] = True

    rgb_m_array = rgb_image
    depth_m_array = depth_image
    # Apply the masks to set values outside the ROI to 0
    rgb_m_array[~roi_mask_rgb] = 255
    depth_m_array[~roi_mask_depth] = 0
    depth_m = o3d.geometry.Image((depth_m_array).astype(np.float32))
    rgb_m = o3d.geometry.Image((rgb_m_array).astype(np.uint8))

    rgbd_m_image = o3d.geometry.RGBDImage.create_from_color_and_depth(rgb_m, depth_m)

    return depth_m, rgb_m, rgbd_m_image

def display_world(world_pcd):
    coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    geometry = [coordFrame]
    geometry.append(world_pcd)
    o3d.visualization.draw_geometries(geometry)

def display_world_nb(world_pcd):
    coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
    geometry = [coordFrame]
    geometry.append(world_pcd)
    draw(geometry)

def get_segment(segments, index, rgbd_image, rsc, type="box", viz_scale=1500.0, method='iterative', display=True):
    '''
    @param segments list of Open3D point cloud segments
    @param index index of segment to display
    @param rgbd_image Open3D RGBD Image
    '''

    if _VERBOSE:
        print( f"Inside `get_segment` @ {os.getpid()}...", flush=True, file=sys.stderr )

    color_copy = copy.deepcopy(rgbd_image.color)
    depth_copy = copy.deepcopy(rgbd_image.depth)
    dm = None
    cpcd = None
    pcaFrame, tmat = None, None
    start = time.time()
    if type == "box" or type == "box-dbscan":
        if _VERBOSE:
            print( f"About to `retrieve_mask_from_image_crop` ...", flush=True, file=sys.stderr )
        dm, rm, imgm = retrieve_mask_from_image_crop(segments[index][0], rgbd_image)
    elif type == "mask":
        dm = create_depth_mask_from_mask(np.array(segments[index][0]), rgbd_image.depth)
    
    if _VERBOSE:
        print( f"About to `crop_and_denoise_pcd` with {dm} ...", flush=True, file=sys.stderr )

    cpcd = crop_and_denoise_pcd(dm, rgbd_image, rsc, NB=5)
    # cpcd = crop_and_denoise_pcd(dm, rgbd_image, rsc, NB=10)
    # cpcd = crop_and_denoise_pcd( dm, rgbd_image, rsc, NB=20 )
    
    if type == "box-dbscan": # much cheaper than SAM
        # find largest cluster with dbscan
        labels = np.array(cpcd.cluster_dbscan(eps=0.04, min_points=50))
        # Find the label with the maximum count
        unique_labels, label_counts = np.unique(labels, return_counts=True)
        largest_cluster_label = unique_labels[np.argmax(label_counts)]
        largest_cluster_points = np.array(cpcd.points)[labels == largest_cluster_label]
        largest_cluster_colors = np.array(cpcd.colors)[labels == largest_cluster_label]
        dbspcd = o3d.geometry.PointCloud()
        dbspcd.points = o3d.utility.Vector3dVector(largest_cluster_points)
        dbspcd.colors = o3d.utility.Vector3dVector(largest_cluster_colors)
        cpcd = dbspcd
    if _VERBOSE:
        print( f"About to `cpcd.compute_mean_and_covariance` ...", flush=True, file=sys.stderr )
    mc = cpcd.compute_mean_and_covariance()
    grasp_pose = [mc[0][1], -mc[0][0], mc[0][2]]
    if method == 'iterative':
        if _VERBOSE:
            print( f"About to `get_pca_frame` ...", flush=True, file=sys.stderr )
        pcaFrame, tmat = get_pca_frame(mc[0], mc[1], scale=viz_scale)
    elif method == 'quat':
        pcaFrame, tmat = get_pca_frame_quat(mc[0], mc[1], scale=viz_scale)
    tmat[:3, 3] = grasp_pose
    if _VERBOSE:
        print( f"About to `o3d.geometry.TriangleMesh.create_coordinate_frame` ...", flush=True, file=sys.stderr )
    worldFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075, origin=[0, 0, 0])
    geometries = [cpcd, pcaFrame, worldFrame]

    if display:
        draw(geometries)

    # despite by my best efforts, I cannot easily make a deep copy of rgbd_image.
    # need to reassign color, depth value copies to rgbd_image
    # in order to view other crops without having to retake an image
    rgbd_image.color = color_copy
    rgbd_image.depth = depth_copy

    return rgbd_image, cpcd, tmat, pcaFrame

# pca helper functions
def find_duplicate_values_and_indices(arr):
    unique_values, unique_indices = np.unique(arr, return_inverse=True)
    counts = np.bincount(unique_indices)
    duplicate_indices = np.where(counts > 1)[0]
    duplicate_values = unique_values[duplicate_indices]
    indices_of_duplicates = [np.where(unique_indices == i)[0] for i in duplicate_indices]
    return duplicate_values, indices_of_duplicates

def contains_duplicates(arr):
    # Count occurrences of each element
    unique, counts = np.unique(arr, return_counts=True)
    # Check if any element has more than one occurrence
    return np.any(counts > 1)

# chatgpt code to align pca with x, y, z axes
def rotation_matrix_to_align_with_axes(evals, evecs):
    """
    Construct a rotation matrix to align the given axes with the standard axes (x, y, and z).

    Parameters:
        axes: list of numpy arrays representing the normalized principal axes

    Returns:
        R: 3x3 numpy array representing the rotation matrix
    """
    '''
    # Example usage:
    axes = [[0.91574736, -0.40073104, 0.02866033],
            [-0.35602909, -0.77640133, 0.52004256],
            [0.18614527, 0.48643151, 0.85365937]]
    R = rotation_matrix_to_align_with_axes(axes)
    '''
    # Get rotation matrices to align each axis with the standard axes
    r = np.eye(3)
    evals_sorted = []
    d = []
    for axis in evecs:
        dot_products = [np.dot(axis, np.array([1, 0, 0])), # x-axis
                        np.dot(axis, np.array([0, 1, 0])), # y-axis
                        np.dot(axis, np.array([0, 0, 1]))] # z-axis
        d.append(dot_products)

    # iterative duplicate removal so that column argmax indices are unique
    # this is really stupid
    arr = np.vstack(np.array(d))
    ai = np.argmax(arr, axis=0)
    while contains_duplicates(ai):
        val, idc = find_duplicate_values_and_indices(ai)
        row_min = np.argmin([arr[val][0][i] for i in idc])
        arr[val, idc[0][row_min]] = -100
        ai = np.argmax(arr, axis=0)
        if _VERBOSE:
            print(f"modified indices: {ai}")

    for i in range(len(evecs)):
        r[:, i] = evecs[ai[i]]
        evals_sorted.append(evals[ai[i]])

    return r, np.array(evals_sorted)


# silly method to turn the pca frame into the desired frame (highest magnitude positive z-axis)
# rather than checking rhr fully, just flip y-axis
# the orientation returned here does not play nice with the robot when used on bounding box pcds
def get_pca_frame(pos, cmat, scale=500.0):
    '''
    @param pos 3d cartesian position of object
    @param cmat 3x3 covariance matrix
    '''
    evals, pca = np.linalg.eig(cmat)
    tmat = np.eye(4)
    rot, evals_sorted = rotation_matrix_to_align_with_axes(evals, pca)

    # check right hand rule
    x, y, z = np.split(rot, 3, axis=1)
    # cross_yz = np.cross(y.T, z.T)
    # cross_xz = np.cross(x.T, z.T)
    # if np.dot(cross_yz, x) < 0:
    #     print("flipping x axis")
    #     x = -x
    # elif np.dot(cross_xz, y) < 0:
    #     print("flipping y axis")
    #     y = -y
    # rot = np.hstack([y, -x, z])
    # flip z-axis if dot product with positive z-axis is negative
    if _VERBOSE:
        print(f"z-axis dot product: {np.dot(z.T, np.array([0, 0, 1]))}")
    if np.dot(z.T, np.array([0, 0, 1])) < 0:
        if _VERBOSE:
            print("flipping z axis")
        z = -z
        # adhere to right hand rule
        if _VERBOSE:
            print("flipping x-axis")
        x = -x

    # flip y-axis for rhr
    rot = np.hstack([x, -y, z])

    # construct transformation matrix and coordinate frame mesh
    tmat[:3, :3] = rot
    # tmat[:3, :3] = pca # unaligned
    tmat[:3, 3] = pos
    pcaFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075)

    # apply scaling transformation from PCA eigenvalues
    scaling_matrix = np.diag([*(scale * evals_sorted), 1])
    # print(scaling_matrix)
    # print(f"tmat: {tmat}")
    # print(f"scaled tmat: {tmat @ scaling_matrix}")
    pcaFrame.transform(tmat @ scaling_matrix)
    # pcaFrame.transform(tmat)
    # print(pcaFrame)
    return pcaFrame, tmat

# quaternion helper functions
def quat_angle(q1, q2):
    '''
    @return angle between q1, q1 in radians
    '''
    return np.arccos(2 * (np.dot(q1, q2))**2 - 1)

# for get_pca_frame_quat, not used
def closest_orientation(evals, evecs):
    '''
    @param evals eigenvalues of covariance matrix
    @param evecs eigenvectors of covariance matrix
    @return rotation matrix, evals with orientation closest to origin coordinate frame
    '''
    # construct three different rotation matrices from three eigenvectors, each evec represents one column
    r1 = np.column_stack(evecs)
    r2 = np.column_stack([evecs[1], evecs[2], evecs[0]])
    r3 = np.column_stack([evecs[2], evecs[0], evecs[1]])
    rs = [r1, r2, r3]

    es2 = [evals[1], evals[2], evals[0]]
    es3 = [evals[2], evals[0], evals[1]] # yea i know this is stupid
    es = [evals, es2, es3]

    r1q = R.as_quat(R.from_matrix(r1))
    r2q = R.as_quat(R.from_matrix(r2))
    r3q = R.as_quat(R.from_matrix(r3))
    uq = R.as_quat(R.from_matrix(np.eye(3))) # unit quaternion
    rqs = [r1q, r2q, r3q]
    # find the quaternion with the smallest angle

    # angles between quaternion and unit quaternion
    angles = [quat_angle(rq, uq) for rq in rqs]
    smallest_angle = np.argmin(angles)

    return rs[smallest_angle], np.array(es[smallest_angle])

# get pca frame closest to world-frame via quat orientation distance
# does not return desired frame as well as silly method (highest magnitude positive z-axis)
def get_pca_frame_quat(pos, cmat, scale=500.0):
    '''
    @param pos 3d cartesian position of object
    @param cmat 3x3 covariance matrix
    '''
    evals, evecs = np.linalg.eig(cmat)
    tmat = np.eye(4)
    rot, evals_sorted = closest_orientation(evals, evecs)
    # print(evals_sorted)
    # check right hand rule
    x, y, z = np.split(rot, 3, axis=1)
    cross_yz = np.cross(y.T, z.T)
    cross_xz = np.cross(x.T, z.T)
    # if np.dot(cross_xz, y) < 0:
    #     print("flipping y axis")
    #     y = -y
    # elif np.dot(cross_yz, x) < 0:
    #     print("flipping x axis")
    #     x = -x
    rot = np.hstack([x, -y, z])

    # construct transformation matrix and coordinate frame mesh
    tmat[:3, :3] = rot
    # tmat[:3, :3] = pca # unaligned
    tmat[:3, 3] = pos
    pcaFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075)

    # apply scaling transformation from PCA eigenvalues
    scaling_matrix = np.diag([*(scale * evals_sorted), 1])
    # print(scaling_matrix)
    # print(tmat)
    # print(tmat @ scaling_matrix)
    pcaFrame.transform(tmat @ scaling_matrix)
    # pcaFrame.transform(tmat)
    # print(pcaFrame)
    return pcaFrame, tmat