########## INIT ####################################################################################

##### Imports #####

### Standard ###
import datetime, sys
from time import sleep

### Special ###
import numpy as np

import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence

### Local ###
from task_planning.utils import row_vec_to_homog
from task_planning.symbols import extract_row_vec_pose
from env_config import _Z_SAFE, _ROBOT_FREE_SPEED, _ROBOT_HOLD_SPEED

sys.path.append( "../" )
from magpie.poses import pose_error
from magpie.BT import Move_Arm, Open_Gripper, Close_Gripper



########## CONSTANTS & COMPONENTS ##################################################################

### Init data structs & Keys ###
_DUMMYPOSE     = np.eye(4)
MP2BB          = dict()  # Hack the BB object into the built-in namespace
SCAN_POSE_KEY  = "scanPoses"
HAND_OBJ_KEY   = "handHas"
PROTO_PICK_ROT = np.array( [[ -1.0,  0.0,  0.0, ],
                            [  0.0,  1.0,  0.0, ],
                            [  0.0,  0.0, -1.0, ]] )
_GRASP_OFFSET_Z = 0.110 + 0.110 # Link 6 to gripper tip
TCP_XFORM = np.array([
    [1, 0, 0, -1.15 / 100     ],
    [0, 1, 0,  1.3 / 100      ],
    [0, 0, 1, _GRASP_OFFSET_Z ],
    [0, 0, 0, 1               ],
])

### Set important BB items ###
MP2BB[ SCAN_POSE_KEY ] = dict()





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



########## BASE CLASS ##############################################################################

class BasicBehavior( Behaviour ):
    """ Abstract class for repetitive housekeeping """
    
    def __init__( self, name = None, ctrl = None ):
        """ Set name to the child class name unless otherwise specified """
        if name is None:
            super().__init__( name = str( self.__class__.__name__  ) )
        else:
            super().__init__( name = name )
        self.ctrl  = ctrl
        self.msg   = ""
        self.logger.debug( f"[{self.name}::__init__()]" )
        if self.ctrl is None:
            self.logger.warning( f"{self.name} is NOT conntected to a robot controller!" )
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
        """ Set root node reference and running parameters """
        self.root   = root
        self.status = Status.INVALID
        self.freq   = tickHz
        self.msg    = ""
        self.Nlim   = int( limit_s * tickHz )
        self.i      = 0


    def setup_BT_for_running( self ):
        """ Prep BT for running """
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


    def tick_once( self ):
        """ Run one simulation step """
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


def grasp_pose_from_obj_pose( rowVec ):
    """ Return the homogeneous coords given [Px,Py,Pz,Ow,Ox,Oy,Oz] """
    rowVec = extract_row_vec_pose( rowVec )
    offVec = TCP_XFORM[0:3,3]
    if len( rowVec ) == 7:
        posn    = rowVec[0:3]
        rtnPose = np.eye(4)
        rtnPose[0:3,0:3] = PROTO_PICK_ROT
        rtnPose[0:3,3]   = posn + offVec
    elif len( rowVec ) == 4:
        rtnPose = np.array( rowVec )
        rtnPose[0:3,0:3] = PROTO_PICK_ROT
        rtnPose[0:3,3]  += offVec
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
            name = f"Move Free from {poseBgn} --to-> {poseEnd}"

        super().__init__( args, robot, name )

        self.poseEnd = extract_row_vec_pose( poseEnd )
                
        self.add_child(
            Move_Arm( self.poseEnd, ctrl = robot, linSpeed = _ROBOT_FREE_SPEED )
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
            Close_Gripper( ctrl = robot  )
        )



class MoveHolding( GroundedAction ):
    """ Move the burdened effector to the given location """
    def __init__( self, args, robot = None, name = None ):

        # ?poseBgn ?poseEnd ?label
        poseBgn, poseEnd, label = args

        if name is None:
            name = f"Move Holding {label} --to-> {poseEnd}"
        super().__init__( args, robot, name )

        poseBgn = grasp_pose_from_obj_pose( extract_row_vec_pose( poseBgn ) )
        poseEnd = grasp_pose_from_obj_pose( extract_row_vec_pose( poseEnd ) )
        psnMid1 = np.array( poseBgn[0:3,3] )
        psnMid2 = np.array( poseEnd[0:3,3] )
        psnMid1[2] = _Z_SAFE
        psnMid2[2] = _Z_SAFE
        poseMd1 = np.eye(4) 
        poseMd2 = np.eye(4)
        poseMd1[0:3,0:3] = PROTO_PICK_ROT
        poseMd2[0:3,0:3] = PROTO_PICK_ROT
        poseMd1[0:3,3] = psnMid1
        poseMd2[0:3,3] = psnMid2
    
        self.add_children( [
            Move_Arm( poseMd1, ctrl = robot, linSpeed = _ROBOT_HOLD_SPEED ),
            Move_Arm( poseMd2, ctrl = robot, linSpeed = _ROBOT_HOLD_SPEED ),
            Move_Arm( poseEnd, ctrl = robot, linSpeed = _ROBOT_HOLD_SPEED ),
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
            Open_Gripper( ctrl = robot  )
        )


########## RESPONSIVE PLANNER BEHAVIOR TREES #######################################################


class PerceiveScene( BasicBehavior ):
    """ Ask the Perception Pipeline to get a reading """

    def __init__( self, args, robot = None, name = None, planner = None ):

        assert planner is not None, "`PerceiveScene` REQUIRES a planner reference!"
        self.planner = planner
        self.args    = args
        
        if name is None:
            name = f"PerceiveScene with planner {type(planner)}"
        super().__init__(  name = name, ctrl = robot )


    def initialise( self ):
        """ Actually Move """
        super().initialise()
        if self.ctrl.p_moving():
            print( f"\n WARN: `PerceiveScene.initialise`: Robot was MOVING at init time: {datetime.now().strftime("%H:%M:%S")}!\n" )
        # self.ctrl.moveL( self.pose, self.linSpeed, self.linAccel, self.asynch )


    def update( self ):
        """ Return true if the target reached """
        if self.ctrl.p_moving():
            print( f"\n`PerceiveScene.initialise`: Robot was MOVING at UPDATE time: {datetime.now().strftime("%H:%M:%S")}!\n" )
            self.status = Status.FAILURE
        else:
            camPose = self.ctrl.get_cam_pose()
            self.planner.phase_1_Perceive( 1, camPose )
            sleep( 0.25 )
            if self.planner.p_belief_dist_OK():
                self.status = Status.SUCCESS
            else:
                self.status = Status.FAILURE
        return self.status





class Interleaved_MoveFree_and_PerceiveScene( GroundedAction ):
    """ Get a replacement sequence for `MoveFree` that stops for perception at the appropriate times """

    def __init__( self, mfBT, planner, sensePeriod_s, name = None ):

        targetP = mfBT.poseEnd.copy()
        
        if name is None:
            name = f"`Interleaved_MoveFree_and_Perceive`, args: {mfBT.args}"
        super().__init__( mfBT.args, mfBT.robot, name )

        # Init #
        self.distMax = _ROBOT_FREE_SPEED * sensePeriod_s
        self.zSAFE = max( _Z_SAFE, targetP[2,3] ) # Eliminate (some) silly vertical movements
        
        # Poses to be Modified at Ticktime #
        self.targetP = targetP
        self.pose1up = _DUMMYPOSE.copy()
        self.pose2up = _DUMMYPOSE.copy()
        
        # Empty sequences to be built at RUNTIME #
        self.moveUp = Sequence( "Leg 1", memory = True )
        self.moveJg = Sequence( "Leg 2", memory = True )
        self.mvTrgt = Sequence( "Leg 3", memory = True )
        
        # 0. WARNING: Start with a sensing action
        self.add_child( PerceiveScene( mfBT.args, robot = mfBT.robot, name = "PerceiveScene 1", planner = planner ) )
        # 1. Move direcly up from the starting pose
        self.add_child( self.moveUp )
        # 2. Translate to above the target
        self.add_child( self.moveJg )
        # 3. Move to the target pose
        self.add_child( self.mvTrgt )


    def initialise( self ):
        """
        ( Ticked first time ) or ( ticked not RUNNING ):
        Generate child sequences with respect to intermittent sensing requirement
        """
        # 1. Fetch pose NOW
        nowPose = self.ctrl.get_tcp_pose()
        
        # 2. Compute intermediate poses
        self.pose1up = nowPose.copy()
        self.pose1up[2, 3] = self.zSAFE

        self.pose2up = self.targetP.copy()
        self.pose2up[2, 3] = self.zSAFE

        # 3. Construct child sequences
        # FIXME: START HERE


########## PLANS ###################################################################################

class Plan( Sequence ):
    """ Special BT `Sequence` with assigned priority, cost, and confidence """

    def __init__( self ):
        """ Set default priority """
        super().__init__( name = "PDDLStream Plan", memory = True )
        self.msg    = "" # --------------- Message: Reason this plan failed -or- OTHER
        self.ctrl   = None
    
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
    """ Fetch the `i`th item from `pdlsPlan` and parameterize a BT that operates on the environment """
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