########## INIT ####################################################################################

##### Imports ####################################
# Numpy
import numpy as np
from numpy import radians

# Spatial Math is used for manipulating geometric primitives
import spatialmath as sm
from spatialmath import SE3

# UR Interface
import rtde_control
import rtde_receive
from rtde_receive import RTDEReceiveInterface as RTDEReceive

# Gripper Interface
import serial.tools.list_ports
from magpie.motor_code import Motors

# Poses is from rmlib and used for converting between 4 x 4 homogenous pose and 6 element vector representation (x,y,z,rx,ry,rz)
from magpie import poses

##### Constants ##################################
from magpie.homog_utils import homog_xform, R_krot

_CAMERA_XFORM = homog_xform( # TCP --to-> Camera
    rotnMatx = R_krot( [0.0, 0.0, 1.0], -np.pi/2.0 ),
    # posnVctr = [0.0, 0.0, 0.084-0.2818]
    posnVctr = [0.0, 0.0, -0.084]
)

########## HELPER FUNCTIONS ########################################################################


def pose_vector_to_homog_coord( poseVec ):
    """ Express the pose vector in homogeneous coordinates """
    # poseVector is a 6 element list of [x, y, z, rX, rY, rZ]
    return poses.pose_vec_to_mtrx( poseVec )


def homog_coord_to_pose_vector( poseMatrix ):
    """ Converts poseMatrix into a 6 element list of [x, y, z, rX, rY, rZ] """
    # poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
    return poses.pose_mtrx_to_vec( np.array( poseMatrix ) )


def get_USB_port_with_desc( descStr ):
    """ Return the name of the first USB port that has `descStr` as a substring of its description, Otherwise return none """
    match = None
    for port, desc, hwid in sorted( serial.tools.list_ports.comports() ):
        if descStr in desc:
            match = port
            break
    return match



########## UR5 INTERFACE ###########################################################################


class UR5_Interface:
    """ Interface class to `ur_rtde` """

    def set_tcp_to_camera_xform( self, xform ):
        """ Set the camera transform """
        self.camXform = np.array( xform )


    def __init__( self, robotIP = "192.168.0.4", cameraXform = None, freq = 500, record=False, record_path=None):
        """ Store connection params and useful constants """
        self.name       = "UR5_CB3"
        self.robotIP    = robotIP # IP address of the robot
        self.ctrl       = None # -- `RTDEControlInterface` object
        self.recv       = None # -- `RTDEReceiveInterface` object
        self.gripper    = None # -- Gripper Controller Interface
        self.Q_safe     = [ radians( elem ) for elem in [ 12.30, -110.36, 95.90, -75.48, -89.59, 12.33 ] ]
        self.torqLim    = 600
        self.freq       = freq
        self.record     = record
        self.record_path = record_path
        self.gripClos_m = 0.002
        self.camXform   = np.eye(4)
        self.z_offset   = 0.01
        self.gripper_offset = [0.012, 0.006, 0.231] # x, y, z offset
        if cameraXform is None:
            self.set_tcp_to_camera_xform( _CAMERA_XFORM )
        else:
            self.set_tcp_to_camera_xform( cameraXform )


    def start( self ):
        """ Connect to RTDE and the gripper """
        self.ctrl = rtde_control.RTDEControlInterface( self.robotIP )
        self.recv = rtde_receive.RTDEReceiveInterface( self.robotIP, self.freq )
        servoPort = get_USB_port_with_desc( "OpenRB" )
        if servoPort is not None:
            self.gripper =  Motors( servoPort )
            self.gripper.torquelimit( self.torqLim )
        else:
            raise RuntimeError( "Could NOT connect to gripper Dynamixel board!" )


    def stop( self ):
        """ Shutdown robot and gripper connections """
        self.ctrl.servoStop()
        self.ctrl.stopScript()
        # self.recv.disconnect()
        # self.ctrl.disconnect()

    def get_name( self ):
        """ Get string that represents this robot """
        return self.name


    def get_joint_angles( self ):
        """ Returns a 6 element numpy array of joint angles (radians) """
        return np.array( self.recv.getActualQ() )


    def get_tcp_pose( self ):
        """ Returns the current pose of the gripper as a SE3 Object (4 x 4 Homegenous Transform) """
        # return sm.SE3( pose_vector_to_homog_coord( self.recv.getActualTCPPose() ) )
        return pose_vector_to_homog_coord( self.recv.getActualTCPPose() )

    def getPose(self):
        # Returns the current pose of the last frame as a SE3 Object (4 x 4 Homegenous Transform)
        p = self.recv.getActualTCPPose()
        poseMatrix = self.poseVectorToMatrix(p)
        T_N = sm.SE3(poseMatrix)   # convert a pose vector to a matrix SE3 object, SE3 --> special euclidean in 3-dimensional space
        # T_N.plot(name="C")
        return T_N    # T_N is a homogenous transform

    def poseVectorToMatrix(self, poseVector):
        # Converts poseVector into an SE3 Object (4 x 4 Homegenous Transform)
        # poseVector is a 6 element list of [x, y, z, rX, rY, rZ]
        T_N = sm.SE3(poses.pose_vec_to_mtrx(poseVector))
        return T_N

    def get_cam_pose( self ):
        """ Returns the current pose of the gripper as a SE3 Object (4 x 4 Homegenous Transform) """
        # return sm.SE3( pose_vector_to_homog_coord( self.recv.getActualTCPPose() ) )
        return np.dot(
            pose_vector_to_homog_coord( self.recv.getActualTCPPose() ),
            self.camXform
            # self.camXform,
            # pose_vector_to_homog_coord( self.recv.getActualTCPPose() )
        )


    def get_sensor_pose_in_robot_frame( self, sensorPose ):
        """ Get a pose obtained from segmentation in the robot frame """
        return np.dot(
            self.get_cam_pose(),
            sensorPose
            # sensorPose,
            # self.get_cam_pose()
        )


    def moveJ( self, qGoal, rotSpeed = 1.05, rotAccel = 1.4, asynch = True, record=None ):
        """ qGoal is a 6 element numpy array of joint angles (radians) """
        # speed is joint velocity (rad/s)
        rec = record if record is not None else self.record
        if rec:
            # record time, joint pos, 6d pose, joint vel
            self.recv.startFileRecording(self.record_path, ["timestamp", "actual_q", "actual_TCP_pose", "actual_qd", "target_q", "target_TCP_pose"])
        self.ctrl.moveJ( list( qGoal ), rotSpeed, rotAccel, asynch )

    def moveL( self, poseMatrix, linSpeed = 0.25, linAccel = 0.5, asynch = True, record=None ):
        """ Moves tool tip pose linearly in cartesian space to goal pose (requires tool pose to be configured) """
        # poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
        # tool pose defined relative to the end of the gripper when closed
        rec = record if record is not None else self.record
        if rec:
            self.recv.startFileRecording(self.record_path, ["timestamp", "actual_q", "actual_TCP_pose", "actual_qd", "target_q", "target_TCP_pose"])
        self.ctrl.moveL( homog_coord_to_pose_vector( poseMatrix ), linSpeed, linAccel, asynch )

    def move_tcp_cartesian(self, poseMatrix, z_offset=None, record=None):
        # move TCP to a position in cartesian space with user-supplied z_offset, ignoring orientation
        # currently the TCP offset is 0, as in we are operating in the wrist frame
        # apply the true tooltip center position, which is the closed MAGPIE gripper (+231mm) offset back to the wrist
        # empirically determined that we shouldn't have changed the x-axis offset, and that the y-was slightly off
        zoff = self.z_offset if z_offset is None else z_offset
        gripper_offset = [0.012, 0.006, 0.231 - zoff] # x, y, z offset
        pos = poseMatrix[:3, 3]
        grasp_pos = pos - gripper_offset
        tmat_offset = np.eye(4) # make identity matrix with grasp_pose as translation
        tmat_offset[:3, 3] = grasp_pos
        wrist = np.array(self.getPose())
        pose = wrist @ tmat_offset
        self.moveL(pose, record=record)

    def move_tcp_cartesian_delta(self, delta, z_offset=0.0, record=False):
        '''
        @param delta: 3 element list of x, y, z offset
        '''
        tmat_offset = np.eye(4)
        print("3d delta: ", delta)
        delta[-1] -= self.z_offset if z_offset is None else z_offset
        tmat_offset[:3, 3] = delta
        # print(f"tmat: {tmat_offset}")
        wrist = np.array(self.getPose())
        print(f"Current pose: {wrist}")
        wrist[:3, 3] = wrist[:3, 3] + delta
        pose = wrist @ tmat_offset
        print(f"Delta to wrist {wrist}")
        # print(f"wrist @ tmat {pose}")
        # self.moveL(pose, record=record)
        self.moveL(wrist, record=record)

    def move_safe( self, rotSpeed = 1.05, rotAccel = 1.4, asynch = True ):
        """ Moves the arm linearly in joint space to home pose """
        self.moveJ( self.Q_safe, rotSpeed, rotAccel, asynch )


    def stop_recording(self):
        if self.record:
            self.recv.stopFileRecording()

    def p_moving( self ):
        """ Return True if the robot is in motion, Otherwise return False """
        return not self.ctrl.isSteady()


    def open_gripper( self ):
        """ Open gripper to the fullest extent """
        self.gripper.openGripper()


    def set_gripper( self, width ):
        """ Computes the servo angles needed for the jaws to be width mm apart """
        # Sends command over serial to the gripper to hold those angles
        self.gripper.position( self.gripper.distance2theta( width * 1000.0 ) )


    def close_gripper( self ):
        """ Set the gripper fingers to near-zero gap """
        self.set_gripper( self.gripClos_m )


    def align_tcp( self, lock_roll = False, lock_pitch = False, lock_yaw = False ):
        """
        Alignes the gripper with the nearest rotational axis (principle cartesian axes).
        Parameters
        ----------
        lock_roll:  bool
        lock_pitch: bool
        lock_yaw:   bool
        """
        pose       = self.get_tcp_pose()
        rot_matrix = pose[0:3, 0:3]
        R          = poses.rotation_mtrx_to_rpy( rot_matrix )
        for i, value in enumerate( R ):
            if i == 0 and lock_pitch:
                continue
            if i == 1 and lock_yaw:
                continue
            if i == 2 and lock_roll:
                continue
            if value > -3.142 and value <= -2.36:
                R[i] = -3.14  # -180
            elif value > -2.36 and value <= -0.79:
                R[i] = -1.57  # -90
            elif value > -0.79 and value <= 0.79:
                R[i] = 0  # 0
            elif value > 0.79 and value <= 2.36:
                R[i] = 1.57  # 90
            elif value > 2.36 and value <= 3.142:
                R[i] = 3.14  # 180
            else:
                raise RuntimeError("`align_tcp`: Encountered an unexpected value!")
        rot_matrix = poses.rpy_to_rotation_mtrx( R )
        pose[0:3, 0:3] = rot_matrix
        self.moveL( pose )
        return pose
