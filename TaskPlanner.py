"""
TaskPlanner.py
Correll Lab, CU Boulder
Contains the Baseline and Responsive Planners described in FIXME: INSERT PAPER REF AND DOI
Version 2024-07
Contacts: {james.watson-2@colorado.edu,}
"""
########## INIT ####################################################################################

##### Imports #####

### Standard ###
import sys, subprocess, time, os
now = time.time
from queue import Queue
from time import sleep
from pprint import pprint
from random import random
from copy import deepcopy
from traceback import print_exc, format_exc
from datetime import datetime



### Special ###
import numpy as np
from py_trees.common import Status

### Local ###
from env_config import ( _BLOCK_SCALE, _MAX_Z_BOUND, _CONFUSE_PROB, _NULL_NAME, _NULL_THRESH, _BLOCK_NAMES, _VERBOSE, 
                         _MIN_X_OFFSET, _MAX_X_OFFSET, _MIN_Y_OFFSET, _MAX_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN,
                         _ACCEPT_POSN_ERR, _MIN_SEP, _POST_N_SPINS, _USE_GRAPHICS, _N_XTRA_SPOTS, )
sys.path.append( "./task_planning/" )
from task_planning.symbols import ObjectReading, ObjPose, extract_row_vec_pose, extract_pose_as_homog
from task_planning.utils import ( get_confusion_matx, get_confused_class_reading, 
                                  DataLogger, pb_posn_ornt_to_row_vec, row_vec_to_pb_posn_ornt, diff_norm, )
from task_planning.actions import display_PDLS_plan, get_BT_plan_until_block_change, BT_Runner
from task_planning.belief import ObjectMemory
sys.path.append( "./vision/" )
from vision.obj_ID_server import Perception_OWLViT
sys.path.append( "./magpie/" )
from magpie import ur5 as ur5
from magpie.poses import repair_pose, translation_diff



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


def p_inside_workspace_bounds( pose ):
    """ Return True if inside the bounding box, Otherwise return False """
    if len( pose ) == 4:
        x = pose[0,3]
        y = pose[1,3]
        z = pose[2,3]
    elif len( pose ) == 7:
        x = pose[0]
        y = pose[1]
        z = pose[2]
    else:
        raise ValueError( f"`p_inside_workspace_bounds`: Input was neither homogeneous nor [Px,Py,Pz,Ow,Ox,Oy,Oz]\n{pose}" )        
    return (_MIN_X_OFFSET <= x <= _MAX_X_OFFSET) and (_MIN_Y_OFFSET <= y <= _MAX_Y_OFFSET) and (0.0 < z <= _MAX_Z_BOUND)


def euclidean_distance_between_symbols( sym1, sym2 ):
    """ Extract pose component from symbols and Return the linear distance between those poses """
    pose1 = extract_pose_as_homog( sym1 )
    pose2 = extract_pose_as_homog( sym2 )
    return translation_diff( pose1, pose2 )


def entropy_factor( probs ):
    """ Return a version of Shannon entropy scaled to [0,1] """
    if isinstance( probs, dict ):
        probs = list( probs.values() )
    tot = 0.0
    for p in probs:
        tot -= p * np.log(p)
    return tot / np.log( len( probs ) )



########## PERCEPTION CONNECTION ###################################################################
# _Z_SNAP_BOOST = 0.25 * _BLOCK_SCALE
_Z_SNAP_BOOST = 0.00

class VisualCortex:
    """ Manages incoming observations """

    def reset_state( self ):
        """ Erase memory components """
        self.scan   = list()
        self.memory = list()


    def get_last_best_readings( self ):
        """ Return the readings in which we believe the most """
        return self.memory[:]
    

    def __init__( self ):
        """ Start OWL-ViT and init object persistence """
        
        # 1. Setup state
        self.reset_state()
        # self.tFresh = 20.0
        
        # 2. Start Perception
        self.perc = Perception_OWLViT
        print( f"`VisualCortex.__init__`: Waiting on OWL-ViT to start ..." )
        self.perc.start_vision()
        print( f"`VisualCortex.__init__`: OWL-ViT STARTED!" )


    def stop( self ):
        """ Stop the Perception Process and the Communication Thread """
        self.perc.shutdown()
        print( "\n##### `VisualCortex` SHUTDOWN #####\n" )


    def snap_z_to_nearest_block_unit_above_zero( self, z ):
        """ SNAP TO NEAREST BLOCK UNIT && SNAP ABOVE TABLE """
        zUnit = np.rint( (z-(_BLOCK_SCALE/2.0)+_Z_SNAP_BOOST) / _BLOCK_SCALE ) # Quantize to multiple of block unit length
        zBloc = (zUnit*_BLOCK_SCALE) + (_BLOCK_SCALE/2.0)
        return zBloc
        # return max( zBloc, _BLOCK_SCALE )


    def observation_to_readings( self, obs, xform = None ):
        """ Parse the Perception Process output struct """
        rtnBel = []
        if xform is None:
            xform = np.eye(4)
        for item in obs.values():
            dstrb = {}
            blcZ  = 0.0
            tScan = item['Time']
            for nam, prb in item['Probability'].items():
                dstrb[ match_name( nam ) ] = prb
            if len( item['Pose'] ) == 16:
                objPose = xform.dot( np.array( item['Pose'] ).reshape( (4,4,) ) ) 
                # HACK: SNAP TO NEAREST BLOCK UNIT && SNAP ABOVE TABLE
                blcZ = self.snap_z_to_nearest_block_unit_above_zero( objPose[2,3] )
                objPose = [ objPose[0,3], objPose[1,3], blcZ, 1,0,0,0, ]
            else:
                # HACK: SNAP TO NEAREST BLOCK UNIT && SNAP ABOVE TABLE
                blcZ = self.snap_z_to_nearest_block_unit_above_zero( item['Pose'][2] )
                objPose = [ objPose[0], objPose[1], blcZ, 1,0,0,0, ]
            rtnBel.append( ObjectReading( labels = dstrb, pose = ObjPose( objPose ), ts = tScan, count = item['Count'] ) )
        return rtnBel
    

    def rectify_readings( self, objReadingList ):
        """ Accept/Reject/Update noisy readings from the system """

        # 1. For every item of incoming object info
        nuMem = list()
        nuSet = set([])
        rmSet = set([])
        totLst = objReadingList[:]
        totLst.extend( self.memory )
        Ntot = len( totLst )
        for r, objR in enumerate( totLst ):
            # HACK: ONLY CONSIDER OBJECTS INSIDE THE WORKSPACE
            if p_inside_workspace_bounds( extract_pose_as_homog( objR ) ):
                # 2. Attempt to quantify how much we trust this reading
                objR.score = (1.0 - entropy_factor( objR.labels )) * objR.count
                # 3. Search for a collision with existing info
                conflict = [objR,]
                for m in range( r+1, Ntot ):
                    objM = totLst[m]
                    if euclidean_distance_between_symbols( objR, objM ) < _MIN_SEP:
                        conflict.append( objM )
                # 4. Sort overlapping indications and add only the top
                conflict.sort( key = lambda item: item.score, reverse = True )
                top    = conflict[0]
                nuHash = id( conflict[0] )
                if (nuHash not in nuSet) and (nuHash not in rmSet):
                    nuMem.append( top )
                    nuSet.add( nuHash )
                    rmSet.update( set( [id(elem) for elem in conflict[1:]] ) )
        # for objM in self.memory:
        #     memHash = id( objM )
        #     if (memHash not in rmSet) and (memHash not in nuSet):
        #         nuMem.append( objM )
        self.memory = nuMem[:]
                    
                    
    def move_reading_from_BT_plan( self, planBT ):
        """ Infer reading to be updated by the robot action, Then update it """
        _verbose = True
        # NOTE: This should run after a BT successfully completes
        # NOTE: This function exits after the first object move
        # NOTE: This function assumes that the reading nearest to the beginning of the 
        updated = False
        dMin    = 1e9
        endMin  = None
        objMtch = None
        
        if planBT.status == Status.SUCCESS:
            for act_i in planBT.children:
                if "MoveHolding" in act_i.__class__.__name__:
                    poseBgn, poseEnd, label = act_i.args
                    for objM in self.memory:
                        dist_ij = euclidean_distance_between_symbols( objM, poseBgn )
                        if (dist_ij <= _MIN_SEP) and (dist_ij < dMin) and (label in objM.labels):
                            dMin    = dist_ij
                            endMin  = poseEnd
                            updated = True
                            objMtch = objM
                    break
            if updated:
                objMtch.pose = endMin
            else:
                if _verbose:
                    print( f"`get_moved_reading_from_BT_plan`: NO update applied by BT {planBT.name}!" )    
        else:
            if _verbose:
                print( f"`get_moved_reading_from_BT_plan`: BT {planBT.name} did NOT complete successfully!" )
        return updated



    def full_scan_noisy( self, xform = None, timeout = 2 ):
        """ Find all of the ROYGBV blocks based on output of Perception Process """
        try:
            dataMsgs  = self.perc.build_model()
            if not len(dataMsgs):
                print( "`VisualCortex.full_scan_noisy`: NO OBJECT DATA!" )
            self.scan = self.observation_to_readings( dataMsgs, xform )

        except KeyboardInterrupt:
            print( "`fresh_scan_noisy`: USER REQUESTED SHUTDOWN" )
            return list()
            
        except Exception as e:
            print( f"`full_scan_noisy`, Scan FAILED: {e}" )
            return list()
        
        return self.scan
        

    # def fresh_scan_noisy( self, xform = None, timeout = 10.0 ):
    #     """ Wait for fresh observations """
    #     scan = self.full_scan_noisy( xform = xform )
    #     bgn  = now()
    #     try:
    #         while (not len( scan )) or ((scan[0].ts - self.tLast) < self.tFresh): # and (not self.PANIC):
    #             elapsed = now() - bgn
    #             if (elapsed > timeout):
    #                 print( f"`fresh_scan_noisy`: TIMEOUT at {elapsed} seconds!" )
    #                 break
    #             else:
    #                 print( f"`fresh_scan_noisy`: Waited {elapsed} seconds for a fresh scan ..." )
    #             scan = self.full_scan_noisy( xform = xform )
    #     except KeyboardInterrupt:
    #         print( "`fresh_scan_noisy`: USER REQUESTED SHUTDOWN" )
    #         return list()
    #     except Exception as e:
    #         print( f"`fresh_scan_noisy`, Scan FAILED: {e}" )
    #         return list()
    #     return scan


########## BASELINE PLANNER ########################################################################

##### Planning Params #####################################################

# _trgtGrn = ObjPose( [ _MIN_X_OFFSET+_X_WRK_SPAN/2.0, _MIN_Y_OFFSET+_Y_WRK_SPAN/2.0, 1.0*_BLOCK_SCALE,  1,0,0,0 ] )
_trgtGrn = ObjPose( [ _MIN_X_OFFSET+_X_WRK_SPAN/2.0, _MIN_Y_OFFSET+_Y_WRK_SPAN/2.0, 0.5*_BLOCK_SCALE,  1,0,0,0 ] )


_temp_home = np.array( [[-1.000e+00, -1.190e-04,  2.634e-05, -2.540e-01],
                        [-1.190e-04,  1.000e+00, -9.598e-06, -4.811e-01],
                        [-2.634e-05, -9.601e-06, -1.000e+00,  4.022e-01],
                        [ 0.000e+00,  0.000e+00,  0.000e+00,  1.000e+00],] )

_GOOD_VIEW_POSE = repair_pose( np.array( [[-0.749, -0.513,  0.419, -0.428,],
                                          [-0.663,  0.591, -0.46 , -0.273,],
                                          [-0.012, -0.622, -0.783,  0.337,],
                                          [ 0.   ,  0.   ,  0.   ,  1.   ,],] ) )

_HIGH_VIEW_POSE = repair_pose( np.array( [[-0.709, -0.455,  0.539, -0.51 ],
                                          [-0.705,  0.442, -0.554, -0.194],
                                          [ 0.014, -0.773, -0.635,  0.332],
                                          [ 0.   ,  0.   ,  0.   ,  1.   ],] ) )



##### Planner #############################################################

# class BaselineTaskPlanner:
class ResponsiveTaskPlanner:
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
        self.grasp   = list()


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
        scan = self.world.full_scan_noisy( xform )
        self.world.rectify_readings( scan )
        self.memory.belief_update( self.world.get_last_best_readings() )


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

        domain_pddl  = read( get_file_path( __file__, os.path.join( 'task_planning/', 'domain.pddl' ) ) )
        stream_pddl  = read( get_file_path( __file__, os.path.join( 'task_planning/', 'stream.pddl' ) ) )
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
            
            ('GraspObj', 'grnBlock' , _trgtGrn  ), # ; Tower
            ('Supported', 'ylwBlock', 'grnBlock'), 
            ('Supported', 'bluBlock', 'ylwBlock'),
            # ('Supported', 'redBlock', 'bluBlock'),

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
        if len( self.grasp ):
            for graspedLabel in self.grasp:
                # [hndl,pDif,bOrn,] = grasped
                # labl = self.world.get_handle_name( hndl )
                rtnFacts.append( ('Holding', graspedLabel,) )
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
                    if ((xySep <= 1.65*_BLOCK_SCALE) and (1.65*_BLOCK_SCALE >= zSep >= 0.9*_BLOCK_SCALE)):
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
        robotPose = ObjPose( self.robot.get_tcp_pose() )
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

        self.perceive_scene( xform ) # We need at least an initial set of beliefs in order to plan
        self.reset_beliefs()
        
        for _ in range( Nscans ):
            self.perceive_scene( xform ) # We need at least an initial set of beliefs in order to plan

        # HACK: REMOVE HALLUCINATED OBJECTS WITH LESSER CONFIDENCE
        self.symbols = self.memory.most_likely_objects( N = 1, cleanDupes = 0, cleanCollision = 0 )
        self.status  = Status.RUNNING

        if _VERBOSE:
            print( f"\nStarting Objects:" )
            for obj in self.symbols:
                print( f"\t{obj}" )


    def allocate_table_swap_space( self, Nspots = _N_XTRA_SPOTS ):
        """ Find some open poses on the table for performing necessary swaps """
        rtnFacts  = []
        freeSpots = []
        occuSpots = [ np.array( sym.pose.pose ) for sym in self.symbols]
        while len( freeSpots ) < Nspots:
            nuPose = pb_posn_ornt_to_row_vec( *rand_table_pose() )
            print( f"\t\tSample: {nuPose}" )
            posn    = nuPose[:3]
            collide = False
            for spot in occuSpots:
                symPosn = spot[:3]
                if diff_norm( posn, symPosn ) < ( _MIN_SEP ):
                    collide = True
                    break
            if not collide:
                freeSpots.append( ObjPose( nuPose ) )
                occuSpots.append( nuPose )
        for objPose in freeSpots:
            rtnFacts.extend([
                ('Waypoint', objPose,),
                ('Free', objPose,),
                ('PoseAbove', objPose, 'table'),
            ])
        return rtnFacts
                

    def phase_2_Conditions( self ):
        """ Get the necessary initial state, Check for goals already met """
        
        if not self.check_goal_objects( self.goal, self.symbols ):
            self.logger.log_event( "Required objects missing", str( self.symbols ) )   
            self.status = Status.FAILURE
        else:
            
            self.facts = [ ('Base', 'table',) ] 

            ## Copy `Waypoint`s present in goals ##
            for g in self.goal[1:]:
                if g[0] == 'GraspObj':
                    self.facts.append( ('Waypoint', g[2],) )
                    if abs(g[2].pose[2] - _BLOCK_SCALE) < _ACCEPT_POSN_ERR:
                        self.facts.append( ('PoseAbove', g[2], 'table') )

            ## Ground the Blocks ##
            for sym in self.symbols:
                self.facts.append( ('Graspable', sym.label,) )

                # blockPose, p_factDex, p_goalDex = self.get_grounded_pose_or_new( sym.pose.pose )
                blockPose = self.get_grounded_fact_pose_or_new( sym.pose.pose )

                # print( f"`blockPose`: {blockPose}" )
                self.facts.append( ('GraspObj', sym.label, blockPose,) )
                if not self.p_grounded_fact_pose( blockPose ):
                    self.facts.append( ('Waypoint', blockPose,) )

            ## Fetch Relevant Facts ##
            self.facts.extend( self.ground_relevant_predicates_noisy() )

            ## Populate Spots for Block Movements ##, 2024-04-25: Injecting this for now, Try a stream later ...
            self.facts.extend( self.allocate_table_swap_space( _N_XTRA_SPOTS ) )

            if _VERBOSE:
                print( f"\n### Initial Symbols ###" )
                for sym in self.facts:
                    print( f"\t{sym}" )
                print()


    def phase_3_Plan_Task( self ):
        """ Attempt to solve the symbolic problem """

        self.task = self.pddlstream_from_problem()

        self.logger.log_event( "Begin Solver" )

        # print( dir( self.task ) )
        if 0:
            print( f"\nself.task.init\n" )
            pprint( self.task.init )
            print( f"\nself.task.goal\n" )
            pprint( self.task.goal )
            print( f"\nself.task.domain_pddl\n" )
            pprint( self.task.domain_pddl )
            print( f"\nself.task.stream_pddl\n" )
            pprint( self.task.stream_pddl )

        try:
            
            solution = solve( 
                self.task, 
                algorithm      = "adaptive", #"focused", #"binding", #"incremental", #"adaptive", 
                unit_costs     = True, # False, #True, 
                unit_efforts   = True, # False, #True,
                reorder        = True,
                initial_complexity = 2,
                # max_complexity = 4,
                # max_failures  = 4,
                # search_sample_ratio = 1/4

            )

            print( "Solver has completed!\n\n\n" )
            print_solution( solution )
            
        except Exception as ex:
            self.logger.log_event( "SOLVER FAULT", format_exc() )
            self.status = Status.FAILURE
            print_exc()
            solution = (None, None, None)
            self.noSoln += 1 # DEATH MONITOR

        plan, cost, evaluations = solution

        if (plan is not None) and len( plan ):
            display_PDLS_plan( plan )
            self.currPlan = plan
            self.action   = get_BT_plan_until_block_change( plan, self.robot )
            self.noSoln   = 0 # DEATH MONITOR
        else:
            self.noSoln += 1 # DEATH MONITOR
            self.logger.log_event( "NO SOLUTION" )
            self.status = Status.FAILURE


    def phase_4_Execute_Action( self ):
        """ Attempt to execute the first action in the symbolic plan """
        
        btr = BT_Runner( self.action, 50.0, 30.0 )
        btr.setup_BT_for_running()

        lastTip = None
        currTip = None

        while not btr.p_ended():
            
            currTip = btr.tick_once()
            if currTip != lastTip:
                self.logger.log_event( f"Behavior: {currTip}", str(btr.status) )
            lastTip = currTip
            
            if (btr.status == Status.FAILURE):
                self.status = Status.FAILURE
                self.logger.log_event( "Action Failure", btr.msg )

        self.logger.log_event( "BT END", str( btr.status ) )

        print( f"Did the BT move a reading?: {self.world.move_reading_from_BT_plan( self.action )}" )
        


    def p_fact_match_noisy( self, pred ):
        """ Search grounded facts for a predicate that matches `pred` """
        for fact in self.facts:
            if pred[0] == fact[0]:
                same = True 
                for i in range( 1, len( pred ) ):
                    if type( pred[i] ) != type( fact[i] ):
                        same = False 
                        break
                    elif isinstance( pred[i], str ) and (pred[i] != fact[i]):
                        same = False
                        break
                    elif (pred[i].index != fact[i].index):
                        same = False
                        break
                if same:
                    return True
        return False

    
    def validate_goal_noisy( self, goal ):
        """ Check if the system believes the goal is met """
        if goal[0] == 'and':
            for g in goal[1:]:
                if not self.p_fact_match_noisy( g ):
                    return False
            return True
        else:
            raise ValueError( f"Unexpected goal format!: {goal}" )


    ##### Task Planner Main Loop ##########################################

    def solve_task( self, maxIter = 100, beginPlanPose = None ):
        """ Solve the goal """
        
        if beginPlanPose is None:
            if _BLOCK_SCALE < 0.030:
                beginPlanPose = _GOOD_VIEW_POSE
            else:
                beginPlanPose = _HIGH_VIEW_POSE

        i = 0

        print( "\n\n\n##### TASK BEGIN #####\n" )

        self.reset_state()
        self.set_goal()
        self.logger.begin_trial()

        indicateSuccess = False

        while (self.status != Status.SUCCESS) and (i < maxIter): # and (not self.PANIC):

            self.robot.moveL( beginPlanPose, asynch = False ) # 2024-07-22: MUST WAIT FOR ROBOT TO MOVE
            # sleep(1)

            print( f"### Iteration {i+1} ###" )
            
            i += 1

            ##### Phase 1 ########################

            print( f"Phase 1, {self.status} ..." )

            self.reset_beliefs() # WARNING: REMOVE FOR RESPONSIVE
            self.reset_state() # - WARNING: REMOVE FOR RESPONSIVE
            self.set_goal()

            camPose = self.robot.get_cam_pose()
            
            self.phase_1_Perceive( 1, camPose )

            if _USE_GRAPHICS:
                self.memory.display_belief_geo()

            ##### Phase 2 ########################

            print( f"Phase 2, {self.status} ..." )
            self.phase_2_Conditions()

            if self.validate_goal_noisy( self.goal ):
                indicateSuccess = True
                self.logger.log_event( "Believe Success", f"Iteration {i}: Noisy facts indicate goal was met!\n{self.facts}" )
                print( f"!!! Noisy success at iteration {i} !!!" )
                self.status = Status.SUCCESS
            else:
                indicateSuccess = False

            if self.status in (Status.SUCCESS, Status.FAILURE):
                print( f"LOOP, {self.status} ..." )
                continue

            ##### Phase 3 ########################

            print( f"Phase 3, {self.status} ..." )
            self.phase_3_Plan_Task()

            # DEATH MONITOR
            if self.noSoln >= self.nonLim:
                self.logger.log_event( "SOLVER BRAINDEATH", f"Iteration {i}: Solver has failed {self.noSoln} times in a row!" )
                break

            if self.p_failed():
                print( f"LOOP, {self.status} ..." )
                continue

            ##### Phase 4 ########################

            print( f"Phase 4, {self.status} ..." )
            self.phase_4_Execute_Action()

            print()

        if 0: #self.PANIC:
            print( "\n\nWARNING: User-requested shutdown or other fault!\n\n" )
            self.logger.end_trial(
                False,
                {'PANIC': True, 'end_symbols' : list( self.symbols ), }
            )
        else:
            self.logger.end_trial(
                indicateSuccess,
                {'end_symbols' : list( self.symbols ) }
            )

        

        self.logger.save( "data/Baseline" )

        print( f"\n##### PLANNER END with status {self.status} after iteration {i} #####\n\n\n" )



########## EXPERIMENT HELPER FUNCTIONS #############################################################

def responsive_experiment_prep( beginPlanPose = None ):
    """ Init system and return a ref to the planner """
    # planner = BaselineTaskPlanner()
    planner = ResponsiveTaskPlanner()
    print( planner.robot.get_tcp_pose() )

    if beginPlanPose is None:
        if _BLOCK_SCALE < 0.030:
            beginPlanPose = _GOOD_VIEW_POSE
        else:
            beginPlanPose = _HIGH_VIEW_POSE

    planner.robot.open_gripper()
    # sleep( OWL_init_pause_s )
    return planner



########## MAIN ####################################################################################
_TROUBLESHOOT = 0
_EXP_BGN_POSE = _HIGH_VIEW_POSE


if __name__ == "__main__":

    dateStr = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")

    if _TROUBLESHOOT:
        print( f"########## Running Debug Code at {dateStr} ##########" )

        # print( repair_pose( _GOOD_VIEW_POSE ) )

        rbt = ur5.UR5_Interface()
        rbt.start()
        # rbt.moveL( repair_pose( _GOOD_VIEW_POSE ) )
        print( f"Began at pose:\n{rbt.get_tcp_pose()}" )
        sleep(1)
        rbt.stop()

    else:
        print( f"########## Running Planner at {dateStr} ##########" )

        try:
            planner = responsive_experiment_prep( _EXP_BGN_POSE )
            planner.solve_task( maxIter = 30, beginPlanPose = _EXP_BGN_POSE )
            sleep( 2.5 )
            planner.shutdown()
            

        except KeyboardInterrupt:
            # User Panic: Attempt to shut down gracefully
            print( f"\nSystem SHUTDOWN initiated by user!, Planner Status: {planner.status}\n" )
            planner.shutdown()

        except Exception as e:
            # Bad Thing: Attempt to shut down gracefully
            print( f"Something BAD happened!: {e}" )
            print_exc()
            print()
            planner.shutdown()

    os.system( 'kill %d' % os.getpid() ) 

    
        