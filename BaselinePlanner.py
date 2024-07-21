########## INIT ####################################################################################

##### Imports #####

### Standard ###
import sys, subprocess, pickle, time
now = time.time
from queue import Queue
from time import sleep
from pprint import pprint
from random import random
from copy import deepcopy
from traceback import print_exc, format_exc



### Special ###
import numpy as np
from py_trees.common import Status

### Local ###
from env_config import ( _BLOCK_SCALE, _N_CLASSES, _CONFUSE_PROB, _NULL_NAME, _NULL_THRESH, _BLOCK_NAMES, _VERBOSE, 
                         _MIN_X_OFFSET, _MAX_X_OFFSET, _MIN_Y_OFFSET, _MAX_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN,
                         _ACCEPT_POSN_ERR, _MIN_SEP, _POST_N_SPINS, _USE_GRAPHICS, _N_XTRA_SPOTS, )
sys.path.append( "./task_planning/" )
from task_planning.symbols import ObjectReading, ObjPose, extract_row_vec_pose, GraspObj
from task_planning.utils import ( multiclass_Bayesian_belief_update, get_confusion_matx, get_confused_class_reading, 
                                  DataLogger, pb_posn_ornt_to_row_vec, row_vec_to_pb_posn_ornt, diff_norm, )
from task_planning.actions import display_PDLS_plan, get_BT_plan_until_block_change, BT_Runner
from task_planning.belief import ObjectMemory
sys.path.append( "./vision/" )
from vision.interprocess import stdioCommWorker
sys.path.append( "./magpie/" )
from magpie import ur5 as ur5
sys.path.append( "./draw_beliefs/" )
from graphics.draw_beliefs import display_belief_geo


### PDDLStream ### 
sys.path.append( "./pddlstream/" )
from pddlstream.utils import read, INF, get_file_path
from pddlstream.language.generator import from_gen_fn, from_test
from pddlstream.language.constants import print_solution, PDDLProblem
from pddlstream.algorithms.meta import solve



########## HELPER FUNCTIONS ########################################################################

def match_name( shortName ):
    """ Search for the environment object name that matches the abbreviated query """
    for envName in _BLOCK_NAMES:
        if shortName in envName:
            return envName
    return _NULL_NAME


def rand_table_pose():
    """ Return a random pose in the direct viscinity if the robot """
    return [ 
        _MIN_X_OFFSET + (_MAX_X_OFFSET-_MIN_X_OFFSET)*random(), 
        _MIN_Y_OFFSET + (_MAX_Y_OFFSET-_MIN_Y_OFFSET)*random(), 
        _BLOCK_SCALE 
    ], [0, 0, 0, 1]



########## PERCEPTION CONNECTION ###################################################################

class VisualCortex:
    """ Mediates communication with the Perception Process """

    def __init__( self, threadUpdateHz = 4.0 ):
        """ Start the Perception Process and the Communication Thread """
        
        # 1. Setup comm `Queue`s
        self.cmndQ = Queue() # Commands     to   Perception Process
        self.dataQ = Queue() # Segmentation from Perception Process
        self.scan  = list()

         # 2. Start the Perception Process
        self.percProc = subprocess.Popen(
            ['python3.9', 'vision/obj_ID_server.py'],
            stdin  = subprocess.PIPE,
            stdout = subprocess.PIPE,
            # stderr = subprocess.PIPE # 2024-07-19: Keep `stderr` open for printing info from the subprocess
        )
        sleep( 0.050 )
        print( f"Process {self.percProc.pid} started with status: {self.percProc.returncode}" )
        sleep( 1 )

        # 3. Start the Communication Thread
        self.commThrd = stdioCommWorker( threadUpdateHz, 
                                         self.cmndQ, self.dataQ, 
                                         self.percProc, self.percProc.stdin, self.percProc.stdout )
        self.commThrd.daemon = True
        self.commThrd.start()
            
        sleep( 1 )


    def stop( self ):
        """ Stop the Perception Process and the Communication Thread """
        for _ in range( 3 ):
            self.cmndQ.put_nowait( {
                'cmnd': "SHUTDOWN",
                'data': None,
            } )
        self.percProc.wait()
        print( f"Process {self.percProc.pid} ended with status: {self.percProc.returncode}" )
        self.commThrd.join( timeout = 1 )
        print( "\n##### `VisualCortex` SHUTDOWN #####\n" )


    def set_effector_pose( self, effHomog ):
        """ Notify the Perception Process of the current effector pose """
        self.cmndQ.put_nowait( {
            'cmnd': "POSE_IN",
            'data': np.array( effHomog ).reshape( (16,) ).tolist(),
        } )


    def observation_to_readings( self, obs, xform = None ):
        """ Parse the Perception Process output struct """
        rtnBel = []
        ts     = now()
        if xform is None:
            xform = np.eye(4)
        for item in obs.values():
            dstrb = {}
            for nam, prb in item['Probability'].items():
                dstrb[ match_name( nam ) ] = prb
            if len( item['Pose'] ) == 16:
                objPose = xform.dot( np.array( item['Pose'] ).reshape( (4,4,) ) ) 
                # HACK: SNAP TO NEAREST BLOCK UNIT
                # blcZ = int((objPose[2,3] - _BLOCK_SCALE/2.0)/_BLOCK_SCALE)*_BLOCK_SCALE + _BLOCK_SCALE/2.0
                blcZ = int((objPose[2,3])/_BLOCK_SCALE)*_BLOCK_SCALE + _BLOCK_SCALE
                objPose = [ objPose[0,3], objPose[1,3], blcZ, 1,0,0,0, ]
            else:
                # HACK: SNAP TO NEAREST BLOCK UNIT
                # blcZ = int((item['Pose'][2] - _BLOCK_SCALE/2.0)/_BLOCK_SCALE)*_BLOCK_SCALE + _BLOCK_SCALE/2.0
                blcZ = int((item['Pose'][2])/_BLOCK_SCALE)*_BLOCK_SCALE + _BLOCK_SCALE
                objPose = [ objPose[0], objPose[1], blcZ, 1,0,0,0, ]
            rtnBel.append( ObjectReading( labels = dstrb, pose = ObjPose( objPose ) ) )
        return rtnBel


    def full_scan_noisy( self, xform = None ):
        """ Find all of the ROYGBV blocks based on output of Perception Process """
        if not self.dataQ.empty():
            dataMsgs = self.dataQ.get_nowait()
            if isinstance( dataMsgs, dict ):
                self.scan = self.observation_to_readings( dataMsgs, xform )
            # FIXME: CONFIRM THE LAST IN THE LIST IS THE MOST RECENT
            elif isinstance( dataMsgs, list ) and isinstance( dataMsgs[-1], dict ):
                self.scan = self.observation_to_readings( dataMsgs[-1], xform )
            else:
                print( "`VisualCortex.full_scan_noisy`: BAD BESSAGE!" )
                pprint( dataMsgs )
                print()
                self.scan = list()
            return deepcopy( self.scan )
        elif (self.scan is not None):
            return deepcopy( self.scan )
        else:
            print( "`VisualCortex.full_scan_noisy`: NO AVAILABLE OBJECT DATA!" )
            return list()



########## BASELINE PLANNER ########################################################################

# FIXME: VERIFY THAT THIS IS A SAFE POSE TO BUILD ON
_trgtGrn = ObjPose( [ _MIN_X_OFFSET+_X_WRK_SPAN/2.0, _MIN_Y_OFFSET+_Y_WRK_SPAN/2.0, 1.0*_BLOCK_SCALE,  1,0,0,0 ] )


class BaselineTaskPlanner:
    """ Basic task planning loop against which the Method is compared """

    ##### Init ############################################################

    def reset_beliefs( self ):
        """ Erase belief memory """
        self.memory  = ObjectMemory() # Distributions over objects
        self.symbols = []
        self.facts   = list()


    def reset_state( self ):
        """ Erase problem state """
        self.status  = Status.INVALID
        self.task    = None
        self.goal    = tuple()


    def __init__( self, world = None ):
        """ Create a pre-determined collection of poses and plan skeletons """
        # NOTE: The planner will start the Perception Process and the UR5 connection as soon as it is instantiated
        self.reset_beliefs()
        self.reset_state()
        self.world  = world if (world is not None) else VisualCortex()
        self.robot  = ur5.UR5_Interface()
        self.logger = DataLogger()
        # DEATH MONITOR
        self.noSoln =  0
        self.nonLim = 10
        self.robot.start()


    def shutdown( self ):
        """ Stop the Perception Process and the UR5 connection """
        self.world.stop()
        self.robot.stop()


    def perceive_scene( self, xform = None ):
        """ Integrate one noisy scan into the current beliefs """
        # FIXME: SEND EFFECTOR POSE
        # FIXME: WHAT IF THE ROBOT IS MOVING?
        self.memory.belief_update( self.world.full_scan_noisy( xform ) )


    ##### Stream Helpers ##################################################

    def get_grounded_pose_or_new( self, rowVec ):
        """ If there is a `Waypoint` approx. to `rowVec`, then return it, Else create new `ObjPose` """
        for fact in self.facts:
            if fact[0] == 'Waypoint' and (diff_norm( rowVec[:3], fact[1].pose[:3] ) <= _ACCEPT_POSN_ERR):
                return fact[1]
        return ObjPose( rowVec )


    def p_grounded_fact_pose( self, poseOrObj ):
        """ Does this exist as a `Waypoint`? """
        rowVec = extract_row_vec_pose( poseOrObj )
        for fact in self.facts:
            if fact[0] == 'Waypoint' and (diff_norm( rowVec[:3], fact[1].pose[:3] ) <= _ACCEPT_POSN_ERR):
                return True
        return False
    

    ##### Stream Creators #################################################

    def get_above_pose_stream( self ):
        """ Return a function that returns poses """

        def stream_func( *args ):
            """ A function that returns poses """

            if _VERBOSE:
                print( f"\nEvaluate ABOVE LABEL stream with args: {args}\n" )

            objcName = args[0]

            for sym in self.symbols:
                if sym.label == objcName:
                    upPose = sym.pose.pose.copy()
                    upPose[2] += _BLOCK_SCALE

                    rtnPose = self.get_grounded_fact_pose_or_new( upPose )
                    print( f"FOUND a pose {rtnPose} supported by {objcName}!" )

                    # rtnPose = ObjPose( upPose )
                    # print( f"FOUND a pose {rtnPose} supported by {objcName}!" )

                    yield (rtnPose,)

        return stream_func
    

    def get_placement_pose_stream( self ):
        """ Return a function that returns poses """

        def stream_func( *args ):
            """ A function that returns poses """

            if _VERBOSE:
                print( f"\nEvaluate PLACEMENT POSE stream with args: {args}\n" )

            # objcName = args[0]
            placed   = False
            testPose = None
            while not placed:
                testPose  = rand_table_pose()
                print( f"\t\tSample: {testPose}" )
                posn , _  = row_vec_to_pb_posn_ornt( testPose )
                collide   = False
                for sym in self.symbols:
                    symPosn, _ = row_vec_to_pb_posn_ornt( sym.pose.pose )
                    if diff_norm( posn, symPosn ) < ( _MIN_SEP ):
                        collide = True
                        break
                if not collide:
                    placed = True
            yield (ObjPose(testPose),)

        return stream_func


    def get_free_placement_test( self ):
        """ Return a function that checks placement poses """

        def test_func( *args ):
            """ a function that checks placement poses """
            print( f"\nEvaluate PLACEMENT test with args: {args}\n" )
            # ?pose
            chkPose   = args[0]
            posn , _  = row_vec_to_pb_posn_ornt( chkPose.pose )
            print( f"Symbols: {self.symbols}" )

            for sym in self.symbols:
                symPosn, _ = row_vec_to_pb_posn_ornt( sym.pose.pose )
                if diff_norm( posn, symPosn ) < ( _MIN_SEP ):
                    print( f"PLACEMENT test FAILURE\n" )
                    return False
            print( f"PLACEMENT test SUCCESS\n" )
            return True
        
        return test_func


    ##### Task Planning Helpers ###########################################

    def pddlstream_from_problem( self ):
        """ Set up a PDDLStream problem with the UR5 """

        domain_pddl  = read( get_file_path( __file__, 'domain.pddl' ) )
        stream_pddl  = read( get_file_path( __file__, 'stream.pddl' ) )
        constant_map = {}
        stream_map = {
            ### Symbol Streams ###
            'sample-above'        : from_gen_fn( self.get_above_pose_stream()     ), 
            # 'sample-free-placment': from_gen_fn( self.get_placement_pose_stream() ), 
            ### Symbol Tests ###
            'test-free-placment': from_test( self.get_free_placement_test() ),
        }

        if _VERBOSE:
            print( "About to create problem ... " )

        return PDDLProblem( domain_pddl, constant_map, stream_pddl, stream_map, self.facts, self.goal )
    

    def set_goal( self ):
        """ Set the goal """

        # FIXME: COLORS AND POSES INCORRECT!

        self.goal = ( 'and',
            
            ('GraspObj', 'grnBlock' , _trgtGrn  ), # ; Tower B
            ('Supported', 'ornBlock', 'grnBlock'), 
            ('Supported', 'vioBlock', 'ornBlock'),
            ('Supported', 'ylwBlock', 'vioBlock'),

            ('HandEmpty',),
        )

        if _VERBOSE:
            print( f"\n### Goal ###" )
            pprint( self.goal )
            print()

    def p_failed( self ):
        """ Has the system encountered a failure? """
        return (self.status == Status.FAILURE)
    

    ##### Noisy Task Monitoring ###########################################

    def get_sampled_block( self, label ):
        """ If a block with `label` was sampled, then return a reference to it, Otherwise return `None` """
        for sym in self.symbols:
            if sym.label == label:
                return sym
        return None
    

    def get_grounded_fact_pose_or_new( self, rowVec ):
        """ If there is a `Waypoint` approx. to `rowVec`, then return it, Else create new `ObjPose` """ 
        for fact in self.facts:
            if fact[0] == 'Waypoint' and (diff_norm( rowVec[:3], fact[1].pose[:3] ) <= _ACCEPT_POSN_ERR):
                return fact[1]
            if fact[0] == 'GraspObj' and (diff_norm( rowVec[:3], fact[2].pose[:3] ) <= _ACCEPT_POSN_ERR):
                return fact[2]
        return ObjPose( rowVec )
    
    
    def ground_relevant_predicates_noisy( self ):
        """ Scan the environment for evidence that the task is progressing, using current beliefs """
        rtnFacts = []
        ## Gripper Predicates ##
        if len( self.world.grasp ):
            for grasped in self.world.grasp:
                [hndl,pDif,bOrn,] = grasped
                labl = self.world.get_handle_name( hndl )
                rtnFacts.append( ('Holding', labl,) )
        else:
            rtnFacts.append( ('HandEmpty',) )
        ## Obj@Loc Predicates ##
        # A. Check goals
        for g in self.goal[1:]:
            if g[0] == 'GraspObj':
                pLbl = g[1]
                pPos = g[2]
                tObj = self.get_sampled_block( pLbl )
                if (tObj is not None) and (diff_norm( pPos.pose[:3], tObj.pose.pose[:3] ) <= _ACCEPT_POSN_ERR):
                    rtnFacts.append( g ) # Position goal met
        # B. No need to ground the rest

        ## Support Predicates && Blocked Status ##
        # Check if `sym_i` is supported by `sym_j`, blocking `sym_j`, NOTE: Table supports not checked
        supDices = set([])
        for i, sym_i in enumerate( self.symbols ):
            for j, sym_j in enumerate( self.symbols ):
                if i != j:
                    lblUp = sym_i.label
                    lblDn = sym_j.label
                    posUp = sym_i.pose
                    posDn = sym_j.pose
                    xySep = diff_norm( posUp.pose[:2], posDn.pose[:2] )
                    zSep  = posUp.pose[2] - posDn.pose[2] # Signed value
                    if ((xySep <= 1.5*_BLOCK_SCALE) and (1.65*_BLOCK_SCALE >= zSep >= _BLOCK_SCALE)):
                        supDices.add(i)
                        rtnFacts.extend([
                            ('Supported', lblUp, lblDn,),
                            ('Blocked', lblDn,),
                            ('PoseAbove', self.get_grounded_fact_pose_or_new( posUp.pose ), lblDn,),
                        ])
        for i, sym_i in enumerate( self.symbols ):
            if i not in supDices:
                rtnFacts.extend( [
                    ('Supported', sym_i.label, 'table',),
                    ('PoseAbove', self.get_grounded_fact_pose_or_new( sym_i.pose.pose ), 'table',),
                ] )
        ## Where the robot at? ##
        robotPose = ObjPose( self.world.robot.get_current_pose() )
        rtnFacts.extend([ 
            ('AtPose', robotPose,),
            ('WayPoint', robotPose,),
        ])
        ## Return relevant predicates ##
        return rtnFacts


    def check_goal_objects( self, goal, symbols ):
        """ Return True if the labels mentioned in the goals are a subset of the determinized symbols """
        goalSet = set([])
        symbSet = set( [sym.label for sym in symbols] )
        for g in goal:
            if isinstance( g, (tuple, list) ):
                prdName = g[0]
                if prdName == 'GraspObj':
                    goalSet.add( g[1] )
                elif prdName == 'Supported':
                    goalSet.add( g[1] )
                    goalSet.add( g[2] )
                else:
                    continue
        return (goalSet <= symbSet)
    

    def block_exists( self, label ):
        """ See if a fact already covers this block """
        for f in self.facts:
            if (f[0] == 'GraspObj') and (f[1] == label):
                return True
        return False


    ##### Task Planning Phases ############################################

    def phase_1_Perceive( self, Nscans = 1, xform = None ):
        """ Take in evidence and form beliefs """

        # FIXME: VERIFY THAT THE CAMERA XFORM IS CORRECT
        
        sleep( 3 )
        
        for _ in range( Nscans ):
            self.perceive_scene( xform ) # We need at least an initial set of beliefs in order to plan

        self.symbols = self.memory.most_likely_objects( N = 1 )
        self.status  = Status.RUNNING

        if _VERBOSE:
            print( f"\nStarting Objects:" )
            for obj in self.symbols:
                print( f"\t{obj}" )



########## MAIN ####################################################################################
if __name__ == "__main__":


    planner = BaselineTaskPlanner()

    sleep( 10 )

    # planner.world.set_effector_pose( planner.robot.get_cam_pose() )
    # planner.world.set_effector_pose( planner.robot.get_cam_pose() )
    # planner.world.set_effector_pose( planner.robot.get_tcp_pose() )
    # planner.world.set_effector_pose( planner.robot.get_tcp_pose() )
    
    sleep( 15 )

    try:
        camPose = planner.robot.get_cam_pose()
        # camPose = planner.robot.get_tcp_pose()
        print( f"\nCamera Pose:\n{camPose}\n" )
        planner.phase_1_Perceive( xform = camPose )
        planner.shutdown()
    except KeyboardInterrupt:
        planner.shutdown()


    display_belief_geo( planner.memory.beliefs )

    sleep( 2.5 )

    

    print( "\n########## PROGRAM END ##########\n" )
        