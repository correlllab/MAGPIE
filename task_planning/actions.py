########## INIT ####################################################################################

##### Imports #####

### Standard ###
import math, sys
from random import random

### Special ###
import numpy as np

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

### Local ###
from task_planning.utils import row_vec_to_homog
from task_planning.symbols import extract_row_vec_pose
from env_config import _Z_SAFE

sys.path.append( "../" )
from magpie.poses import pose_error
from magpie.BT import Move_Arm, Open_Gripper, Close_Gripper

##### Constants #####

_HAND_WAIT   = 100
_GRASP_PAUSE = False
_GRASP_STALL = 4



########## BEHAVIOR HELPERS ########################################################################

def pass_msg_up( bt, failBelow = False ):
    if bt.parent is not None:
        if bt.status == Status.FAILURE:
            if (bt.parent.status != Status.FAILURE) or (len( bt.parent.msg ) == 0):
                setattr( bt.parent, "msg", bt.msg )
                pass_msg_up( bt.parent, True )
            else:
                pass_msg_up( bt.parent )
        elif failBelow:
            setattr( bt.parent, "msg", bt.msg )
            pass_msg_up( bt.parent, True )


def connect_BT_to_robot( bt, robot ):
    """ Assign `robot` controller to `bt` and recursively to all nodes below """
    if hasattr( bt, 'ctrl' ):
        bt.ctrl = robot
    if len( bt.children ):
        for child in bt.children:
            connect_BT_to_robot( child, robot )


def connect_BT_to_world( bt, world ):
    """ Assign `world` environment to `bt` and recursively to all nodes below """
    if hasattr( bt, 'world' ):
        bt.world = world
    if len( bt.children ):
        for child in bt.children:
            connect_BT_to_world( child, world )


def connect_BT_to_robot_world( bt, robot, world ):
    """ Set both controller and environment for this behavior and all children """
    connect_BT_to_robot( bt, robot )
    connect_BT_to_world( bt, world )



########## BASE CLASS ##############################################################################

class BasicBehavior( Behaviour ):
    """ Abstract class for repetitive housekeeping """
    
    def __init__( self, name = None, ctrl = None, world = None ):
        """ Set name to the child class name unless otherwise specified """
        if name is None:
            super().__init__( name = str( self.__class__.__name__  ) )
        else:
            super().__init__( name = name )
        self.ctrl  = ctrl
        self.world = world
        self.msg   = ""
        self.logger.debug( f"[{self.name}::__init__()]" )
        if self.ctrl is None:
            self.logger.warning( f"{self.name} is NOT conntected to a robot controller!" )
        if self.world is None:
            self.logger.warning( f"{self.name} is NOT conntected to a world object!" )
        self.count = 0
        

    def setup(self):
        """ Virtual setup for base class """
        self.logger.debug( f"[{self.name}::setup()]" )          
        
        
    def initialise( self ):
        """ Run first time behaviour is ticked or not RUNNING.  Will be run again after SUCCESS/FAILURE. """
        self.status = Status.RUNNING # Do not let the behavior idle in INVALID
        self.logger.debug( f"[{self.name}::initialise()]" ) 
        self.count = 0         

        
    def terminate( self, new_status ):
        """ Log how the behavior terminated """
        self.status = new_status
        self.logger.debug( f"[{self.name}::terminate().terminate()][{self.status}->{new_status}]" )
        
        
    def update( self ):
        """ Return true in all cases """
        self.status = py_trees.common.Status.SUCCESS
        return self.status
    

    def stall( self, Nwait ):
        """ Run at least `Nwait` ticks """
        rtnStat = Status.INVALID
        if self.count < Nwait:
            rtnStat = Status.RUNNING
        else:
            rtnStat = Status.SUCCESS
        self.count += 1
        return rtnStat
    
    
    
########## CONSTANTS & COMPONENTS ##################################################################

### Init data structs & Keys ###
_DUMMYPOSE     = np.eye(4)
MP2BB          = dict()  # Hack the BB object into the built-in namespace
SCAN_POSE_KEY  = "scanPoses"
HAND_OBJ_KEY   = "handHas"
# PROTO_PICK_ROT = np.array( [[ 0.0,  1.0,  0.0, ],
#                             [ 1.0,  0.0,  0.0, ],
#                             [ 0.0,  0.0, -1.0, ]] )
PROTO_PICK_ROT = np.array( [[ -1.0,  1.0,  0.0, ],
                            [  0.0,  1.0,  0.0, ],
                            [  0.0,  0.0, -1.0, ]] )

### Set important BB items ###
MP2BB[ SCAN_POSE_KEY ] = dict()



########## BASIC BEHAVIORS #########################################################################

### Constants ###
LIBBT_TS_S       = 0.25
DEFAULT_TRAN_ERR = 0.010 # 0.002
DEFAULT_ORNT_ERR = 3*np.pi/180.0


    


########## HELPER FUNCTIONS ########################################################################

def display_PDLS_plan( plan ):
    print( f"\nPlan output from PDDLStream:" )
    if plan is not None:
        for i, action in enumerate( plan ):
            # print( dir( action ) )
            print( f"\t{i+1}: { action.__class__.__name__ }, {action.name}" )
            for j, arg in enumerate( action.args ):
                print( f"\t\tArg {j}:\t{type( arg )}, {arg}" )
    else:
        print( plan )



########## BT-PLANNER INTERFACE ####################################################################

class BT_Runner:
    """ Run a BT with checks """

    def __init__( self, root, tickHz = 4.0, limit_s = 20.0 ):
        """ Set root node and world reference """
        self.root   = root
        # self.world  = world
        self.status = Status.INVALID
        self.freq   = tickHz
        # self.Nstep  = int( max(1.0, math.ceil((1.0 / tickHz) / world.period)))
        self.msg    = ""
        self.Nlim   = int( limit_s * tickHz )
        self.i      = 0


    def setup_BT_for_running( self ):
        """ Connect the plan to world and robot """
        # connect_BT_to_robot_world( self.root, self.world.robot, self.world )
        self.root.setup_with_descendants()


    def display_BT( self ):
        """ Draw the BT along with the status of all the nodes """
        print( py_trees.display.unicode_tree( root = self.root, show_status = True ) )


    def p_ended( self ):
        """ Has the BT ended? """
        return self.status in ( Status.FAILURE, Status.SUCCESS )
    

    def set_fail( self, msg = "DEFAULT MSG: STOPPED" ):
        """ Handle external signals to halt BT execution """
        self.status = Status.FAILURE
        self.msg    = msg
        # self.world.robot_release_all()
        # self.world.spin_for( 250 )


    def tick_once( self ):
        """ Run one simulation step """
        ## Let sim run ##
        # self.world.spin_for( self.Nstep )
        ## Advance BT ##
        if not self.p_ended():
            self.root.tick_once()
        self.status = self.root.status
        self.i += 1
        ## Check Conditions ##
        if (self.i >= self.Nlim) and (not self.p_ended()):
            self.set_fail( "BT TIMEOUT" )
        if self.p_ended():
            pass_msg_up( self.root )
            if len( self.msg ) == 0:
                self.msg = self.root.msg
            self.display_BT() 
        return self.root.tip().name



########## BLOCKS DOMAIN HELPER FUNCTIONS ##########################################################
_GRASP_OFFSET_Z = 0.110 + 0.120

def grasp_pose_from_obj_pose( rowVec ):
    """ Return the homogeneous coords given [Px,Py,Pz,Ow,Ox,Oy,Oz] """
    if len( rowVec ) == 7:
        posn    = rowVec[0:3]
        rtnPose = np.eye(4)
        rtnPose[0:3,0:3] = PROTO_PICK_ROT
        rtnPose[0:3,3]   = posn
        rtnPose[2,3]    += _GRASP_OFFSET_Z
    elif len( rowVec ) == 4:
        rtnPose = np.array( rowVec )
        rtnPose[0:3,0:3] = PROTO_PICK_ROT
        rtnPose[2,3]    += _GRASP_OFFSET_Z
    else:
        print( f"`grasp_pose_from_obj_pose`: UNEXPECTED POSE FORMAT:\n{rowVec}" )
        rtnPose = None
    return rtnPose



########## BLOCKS DOMAIN BEHAVIOR TREES ############################################################

class GroundedAction( Sequence ):
    """ This is the parent class for all actions available to the planner """

    def __init__( self, args = None, robot = None, name = "Grounded Sequence" ):
        """ Init BT """
        super().__init__( name = name, memory = True )
        self.args    = args if (args is not None) else list() # Symbols involved in this action
        self.symbols = list() #  Symbol on which this behavior relies
        self.msg     = "" # ---- Message: Reason this action failed -or- OTHER
        self.ctrl    = robot # - Agent that executes

    def __repr__( self ):
        """ Get the name, Assume child classes made it sufficiently descriptive """
        return str( self.name )
    


class MoveFree( GroundedAction ):
    """ Move the unburdened effector to the given location """
    def __init__( self, args, robot = None, name = None ):

        # ?poseBgn ?poseEnd
        poseBgn, poseEnd = args
        poseEnd = grasp_pose_from_obj_pose( poseEnd )

        if name is None:
            name = f"Move Free from {poseBgn.pose} --to-> {poseEnd.pose}"

        super().__init__( args, robot, name )

        poseEnd = extract_row_vec_pose( poseEnd )
                
        self.add_child(
            # Move_Effector( poseEnd, ctrl = robot )
            Move_Arm( poseEnd, ctrl = robot )
        )



class Pick( GroundedAction ):
    """ Add object to the gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?prevSupport
        label, pose, prevSupport = args
        
        if name is None:
            name = f"Pick {label} at {pose.pose} from {prevSupport}"
        super().__init__( args, robot, name )

        self.add_child( 
            # Grasp( label, pose, name = name, ctrl = robot )
            Close_Gripper( ctrl = robot  )
        )



class Unstack( GroundedAction ):
    """ Add object to the gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?prevSupport
        label, pose, prevSupport = args
        
        if name is None:
            name = f"Unstack {label} at {pose.pose} from {prevSupport}"
        super().__init__( args, robot, name )

        self.add_child( 
            # Grasp( label, pose, name = name, ctrl = robot )
            Close_Gripper( ctrl = robot  )
        )



class MoveHolding( GroundedAction ):
    """ Move the burdened effector to the given location """
    def __init__( self, args, robot = None, name = None ):

        # ?poseBgn ?poseEnd ?label
        poseBgn, poseEnd, label = args

        if name is None:
            name = f"Move Holding {label} --to-> {poseEnd.pose}"
        super().__init__( args, robot, name )

        poseBgn = grasp_pose_from_obj_pose( extract_row_vec_pose( poseBgn ) )
        poseEnd = grasp_pose_from_obj_pose( extract_row_vec_pose( poseEnd ) )
        posnBgn = poseBgn[0:3,3]
        posnEnd = poseEnd[0:3,3]
        psnMid1 = np.array( posnBgn )
        psnMid2 = np.array( posnEnd )
        psnMid1[2] = _Z_SAFE
        psnMid2[2] = _Z_SAFE
        # poseMd1 = psnMid1.tolist() + [1,0,0,0]
        # poseMd2 = psnMid2.tolist() + [1,0,0,0]
        poseMd1 = np.eye(4) 
        poseMd2 = np.eye(4)
        poseMd1[0:3,0:3] = PROTO_PICK_ROT
        poseMd2[0:3,0:3] = PROTO_PICK_ROT
        poseMd1[0:3,3] = psnMid1
        poseMd2[0:3,3] = psnMid2
    
        self.add_children( [
            # Move_Effector( poseMd1, ctrl = robot ),
            # Move_Effector( poseMd2, ctrl = robot ),
            # Move_Effector( poseEnd, ctrl = robot ),
            Move_Arm( poseMd1, ctrl = robot ),
            Move_Arm( poseMd2, ctrl = robot ),
            Move_Arm( grasp_pose_from_obj_pose( poseEnd ), ctrl = robot ),
        ] )



class Place( GroundedAction ):
    """ Let go of gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?label ?pose ?support
        label, pose, support = args
        
        if name is None:
            name = f"Place {label} at {pose.pose} onto {support}"
        super().__init__( args, robot, name )

        self.add_child( 
            # Ungrasp( name = name, ctrl = robot )
            Open_Gripper( ctrl = robot  )
        )


class Stack( GroundedAction ):
    """ Let go of gripper payload """
    def __init__( self, args, robot = None, name = None ):

        # ?labelUp ?poseUp ?labelDn
        labelUp, poseUp, labelDn = args
        
        if name is None:
            name = f"Stack {labelUp} at {poseUp.pose} onto {labelDn}"
        super().__init__( args, robot, name )

        self.add_child( 
            # Ungrasp( name = name, ctrl = robot )
            Open_Gripper( ctrl = robot  )
        )



########## PLANS ###################################################################################

class Plan( Sequence ):
    """ Special BT `Sequence` with assigned priority, cost, and confidence """

    def __init__( self ):
        """ Set default priority """
        super().__init__( name = "PDDLStream Plan", memory = True )
        self.msg    = "" # --------------- Message: Reason this plan failed -or- OTHER
        self.ctrl   = None
        self.world  = None
    
    def __len__( self ):
        """ Return the number of children """
        return len( self.children )

    def append( self, action ):
        """ Add an action """
        self.add_child( action )
    
    def __repr__( self ):
        """ String representation of the plan """
        return f"<{self.name}, Status: {self.status}>"



########## PDLS --TO-> BT ##########################################################################

def get_ith_BT_action_from_PDLS_plan( pdlsPlan, i, robot ):
    """ Fetch the `i`th item from `pdlsPlan` and parameterize a BT that operates on the `world` """
    actName  = pdlsPlan[i].name
    actArgs  = pdlsPlan[i].args
    btAction = None
    if actName == "move_free":
        btAction = MoveFree( actArgs, robot = robot )
    elif actName == "pick":
        btAction = Pick( actArgs, robot = robot )
    elif actName == "unstack":
        btAction = Unstack( actArgs, robot = robot )
    elif actName == "move_holding":
        btAction = MoveHolding( actArgs, robot = robot )
    elif actName == "place":
        btAction = Place( actArgs, robot = robot )
    elif actName == "stack":
        btAction = Stack( actArgs, robot = robot )
    else:
        raise NotImplementedError( f"There is no BT procedure defined for a PDDL action named {actName}!" )
    print( f"Action {i+1}, {actName} --> {btAction.name}, planned!" )
    return btAction


def get_BT_plan_until_block_change( pdlsPlan, robot ):
    """ Translate the PDLS plan to one that can be executed by the robot """
    rtnBTlst = []
    if pdlsPlan is not None:
        for i in range( len( pdlsPlan ) ):
            btAction = get_ith_BT_action_from_PDLS_plan( pdlsPlan, i, robot )
            rtnBTlst.append( btAction )
            if btAction.__class__ in ( Place, Stack ):
                break
    rtnPlan = Plan()
    rtnPlan.add_children( rtnBTlst )
    return rtnPlan