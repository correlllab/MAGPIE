########## INIT ####################################################################################

# Numpy
import numpy as np
from numpy import radians

# Spatial Math is used for manipulating geometric primitives
import spatialmath as sm
from spatialmath import SE3

# UR Interface
import rtde_control
import rtde_receive
# Gripper Interface 
import serial.tools.list_ports
from magpie.Motor_Code import Motors

# Poses is from rmlib and used for converting between 4 x 4 homogenous pose and 6 element vector representation (x,y,z,rx,ry,rz)
from magpie import poses



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
    
    def __init__( self, robotIP = "192.168.0.6" ):
        """ Store connection params and useful constants """
        self.robotIP    = robotIP # IP address of the robot
        self.ctrl       = None # -- `RTDEControlInterface` object 
        self.recv       = None # -- `RTDEReceiveInterface` object 
        self.gripper    = None # -- Gripper Controller Interface
        self.Q_safe     = [ radians( elem ) for elem in [ 12.30, -110.36, 95.90, -75.48, -89.59, 12.33 ] ]
        self.torqLim    = 600
        self.gripClos_m = 0.002
        
        
    def start( self ):
        """ Connect to RTDE and the gripper """
        self.ctrl = rtde_control.RTDEControlInterface( self.robotIP )
        self.recv = rtde_receive.RTDEReceiveInterface( self.robotIP )
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
        
        
    def get_joint_angles( self ):
        """ Returns a 6 element numpy array of joint angles (radians) """
        return np.array( self.recv.getActualQ() )
    
    
    def get_tcp_pose( self ):
        """ Returns the current pose of the gripper as a SE3 Object (4 x 4 Homegenous Transform) """
        # return sm.SE3( pose_vector_to_homog_coord( self.recv.getActualTCPPose() ) )
        return pose_vector_to_homog_coord( self.recv.getActualTCPPose() )
    
    
    def moveJ( self, qGoal, rotSpeed = 1.05, rotAccel = 1.4, asynch = True ):
        """ qGoal is a 6 element numpy array of joint angles (radians) """
        # speed is joint velocity (rad/s)
        self.ctrl.moveJ( list( qGoal ), rotSpeed, rotAccel, asynch )
    
    
    def moveL( self, poseMatrix, linSpeed = 0.25, linAccel = 0.5, asynch = True ):
        """ Moves tool tip pose linearly in cartesian space to goal pose (requires tool pose to be configured) """
        # poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
        # tool pose defined relative to the end of the gripper when closed
        self.ctrl.moveL( homog_coord_to_pose_vector( poseMatrix ), linSpeed, linAccel, asynch )
    
    
    def move_safe( self, rotSpeed = 1.05, rotAccel = 1.4, asynch = True ):
        """ Moves the arm linearly in joint space to home pose """
        self.moveJ( self.Q_safe, rotSpeed, rotAccel, asynch )
    
    
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
        