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
import sys, time, os
now = time.time
from time import sleep
from pprint import pprint
from random import random
from traceback import print_exc, format_exc
from datetime import datetime
from math import isnan



### Special ###
import numpy as np
from py_trees.common import Status
from py_trees.composites import Sequence
from magpie.BT import Open_Gripper
import open3d as o3d

### Local ###
from env_config import ( _BLOCK_SCALE, _SCORE_DECAY_TAU_S, _NULL_NAME, _OBJ_TIMEOUT_S, _BLOCK_NAMES, _VERBOSE, 
                         _MIN_X_OFFSET, _MIN_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN,
                         _ACCEPT_POSN_ERR, _MIN_SEP, _USE_GRAPHICS, _N_XTRA_SPOTS, _MAX_UPDATE_RAD_M, _UPDATE_PERIOD_S,
                         _BT_UPDATE_HZ, _BT_ACT_TIMEOUT_S, _LKG_SEP, )
sys.path.append( "./task_planning/" )
from task_planning.symbols import ( ObjectReading, ObjPose, GraspObj, extract_pose_as_homog, 
                                    euclidean_distance_between_symbols, p_symbol_inside_workspace_bounds )
from task_planning.utils import ( DataLogger, diff_norm, breakpoint, )
from task_planning.actions import ( display_PDLS_plan, get_BT_plan_until_block_change, BT_Runner, 
                                    Interleaved_MoveFree_and_PerceiveScene, MoveFree, GroundedAction, )
from task_planning.belief import ObjectMemory
sys.path.append( "./vision/" )
from vision.obj_ID_server import Perception_OWLViT
sys.path.append( "./graphics/" )
from graphics.draw_beliefs import generate_belief_geo
from graphics.homog_utils import posn_from_xform
sys.path.append( "./magpie/" )
from magpie import ur5 as ur5
from magpie.poses import repair_pose, translation_diff, vec_unit


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
    rtnPose = np.eye(4)
    rtnPose[0:3,3] = [ 
        _MIN_X_OFFSET + 0.5*_X_WRK_SPAN*random(), 
        _MIN_Y_OFFSET + 0.5*_Y_WRK_SPAN*random(), 
        _BLOCK_SCALE/2.0,
    ]
    return rtnPose


def display_belief_geo( beliefList ):
    geo = generate_belief_geo( beliefList )
    o3d.visualization.draw_geometries( geo )


def entropy_factor( probs ):
    """ Return a version of Shannon entropy scaled to [0,1] """
    if isinstance( probs, dict ):
        probs = list( probs.values() )
    tot = 0.0
    # N   = 0
    for p in probs:
        if p > 0.0:
            tot -= p * np.log(p)
            # N   += 1
    return tot / np.log( len( probs ) )


def copy_readings_as_LKG( readLst ):
    """ Return a list of readings intended for the Last-Known-Good collection """
    rtnLst = list()
    for r in readLst:
        rtnLst.append( r.copy_as_LKG() )
    return rtnLst


def copy_readings( readLst ):
    """ Return a list of readings intended for the Last-Known-Good collection """
    rtnLst = list()
    for r in readLst:
        rtnLst.append( r.copy() )
    return rtnLst



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
    

    def __init__( self, noViz = False ):
        """ Start OWL-ViT and init object persistence """
        
        # 1. Setup state
        self.reset_state()
        # self.tFresh = 20.0
        
        # 2. Start Perception
        self.noViz = noViz
        if not noViz:
            self.perc = Perception_OWLViT
            print( f"`VisualCortex.__init__`: Waiting on OWL-ViT to start ..." )
            self.perc.start_vision()
            print( f"`VisualCortex.__init__`: OWL-ViT STARTED!" )
        else:
            self.perc = None
            print( f"`VisualCortex.__init__`: Perception is INACTIVE!" )


    def stop( self ):
        """ Stop the Perception Process and the Communication Thread """
        if not self.noViz:
            self.perc.shutdown()
        print( "\n##### `VisualCortex` SHUTDOWN #####\n" )


    def snap_z_to_nearest_block_unit_above_zero( self, z ):
        """ SNAP TO NEAREST BLOCK UNIT && SNAP ABOVE TABLE """
        sHalf = (_BLOCK_SCALE/2.0)
        zUnit = np.rint( (z-sHalf+_Z_SNAP_BOOST) / _BLOCK_SCALE ) # Quantize to multiple of block unit length
        zBloc = max( (zUnit*_BLOCK_SCALE)+sHalf, sHalf )
        return zBloc
        # return max( zBloc, _BLOCK_SCALE )


    def observation_to_readings( self, obs, xform = None ):
        """ Parse the Perception Process output struct """
        rtnBel = []
        if xform is None:
            xform = np.eye(4)
        for item in obs.values():
            dstrb = {}
            tScan = item['Time']

            for nam, prb in item['Probability'].items():
                dstrb[ match_name( nam ) ] = prb
            if _NULL_NAME not in dstrb:
                dstrb[ _NULL_NAME ] = 0.0

            if len( item['Pose'] ) == 16:
                objPose = xform.dot( np.array( item['Pose'] ).reshape( (4,4,) ) ) 
                # HACK: SNAP TO NEAREST BLOCK UNIT && SNAP ABOVE TABLE
                objPose[2,3] = self.snap_z_to_nearest_block_unit_above_zero( objPose[2,3] )
            else:
                raise ValueError( f"`observation_to_readings`: BAD POSE FORMAT!\n{item['Pose']}" )
            
            # Attempt to quantify how much we trust this reading
            score_i = (1.0 - entropy_factor( dstrb )) * item['Count']
            if isnan( score_i ):
                print( f"\nWARN: Got a NaN score with count {item['Count']} and distribution {dstrb}\n" )
                score_i = 0.0

            rtnBel.append( ObjectReading( labels = dstrb, pose = ObjPose( objPose ), ts = tScan, count = item['Count'], score = score_i ) )
        return rtnBel
    

    def rectify_readings( self, objReadingList, suppressStorage = False, useTimeout = True ):
        """ Accept/Reject/Update noisy readings from the system """
        tCurr = now()
        nuMem = list()
        nuSet = set([])
        rmSet = set([])
        totLst = objReadingList[:]
        if not suppressStorage:
            totLst.extend( self.memory )
        Ntot = len( totLst )
        # 1. For every item of [incoming object info + previous info]
        for r, objR in enumerate( totLst ):
            
            # HACK: ONLY CONSIDER OBJECTS INSIDE THE WORKSPACE
            if not p_symbol_inside_workspace_bounds( extract_pose_as_homog( objR ) ):
                continue

            if (useTimeout and ((tCurr - objR.ts) > _OBJ_TIMEOUT_S)):
                continue

            # 3. Search for a collision with existing info
            conflict = [objR,]
            for m in range( r+1, Ntot ):
                objM = totLst[m]
                if euclidean_distance_between_symbols( objR, objM ) < _LKG_SEP:
                    conflict.append( objM )
            # 4. Sort overlapping indications and add only the top
            conflict.sort( key = lambda item: item.score, reverse = True )
            top    = conflict[0]
            nuHash = id( conflict[0] )
            if (nuHash not in nuSet) and (nuHash not in rmSet):
                nuMem.append( top )
                nuSet.add( nuHash )
                rmSet.update( set( [id(elem) for elem in conflict[1:]] ) )
        if not suppressStorage:
            self.memory = nuMem[:]
        return nuMem
    

    def report_LKG( self ):
        print( f"\n### LKG has {len(self.memory)} items ###" )
        for lkg in self.memory:
            print( lkg )
        print( f"### Done ###\n" )
                    
                    
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
                objMtch.ts   = now() # 2024-07-27: THIS IS EXTREMELY IMPORTANT ELSE THIS READING DIES --> BAD BELIEFS
                # 2024-07-27: NEED TO DO SOME DEEP THINKING ABOUT THE FRESHNESS OF RELEVANT FACTS
                if _verbose:
                    print( f"`get_moved_reading_from_BT_plan`: BT {planBT.name} updated {objMtch}!" )  
            else:
                if _verbose:
                    print( f"`get_moved_reading_from_BT_plan`: NO update applied by BT {planBT.name}!" )    
        else:
            if _verbose:
                print( f"`get_moved_reading_from_BT_plan`: BT {planBT.name} did NOT complete successfully!" )
        return updated


    def get_perc_output( self ):
        """ Return the perception pipeline output without processing or filtering """
        return self.perc.build_model()


    def full_scan_noisy( self, xform = None, observations = None ):
        """ Find all of the ROYGBV blocks based on output of Perception Process """
        try:
            if observations is None:
                dataMsgs = self.perc.build_model()
            elif isinstance( observations, list ):
                dataMsgs = observations[:]
            if not len( dataMsgs ):
                print( "`VisualCortex.full_scan_noisy`: NO OBJECT DATA!" )
            self.scan = self.observation_to_readings( dataMsgs, xform )

        except KeyboardInterrupt as k:
            print( "`fresh_scan_noisy`: USER REQUESTED SHUTDOWN" )
            raise k
            
        except Exception as e:
            print( f"`full_scan_noisy`, Scan FAILED: {e}" )
            return list()
        
        return self.scan
        


########## BASELINE PLANNER ########################################################################

##### Planning Params #####################################################

_poseGrn = np.eye(4)
_poseGrn[0:3,3] = [ _MIN_X_OFFSET+_X_WRK_SPAN/2.0, _MIN_Y_OFFSET+_Y_WRK_SPAN/2.0, 0.5*_BLOCK_SCALE, ]
_trgtGrn = ObjPose( _poseGrn )


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

_HIGH_TWO_POSE = repair_pose( np.array( [[-0.351, -0.552,  0.756, -0.552],
                                         [-0.936,  0.194, -0.293, -0.372],
                                         [ 0.015, -0.811, -0.585,  0.283],
                                         [ 0.   ,  0.   ,  0.   ,  1.   ],] ) )



##### Planner #############################################################

class ResponsiveTaskPlanner:
    """ Basic task planning loop against which the Method is compared """

    ##### Init ############################################################

    def reset_beliefs( self ):
        """ Erase belief memory """
        self.memory  = ObjectMemory() # Distributions over objects
        self.beliefs = list() # ------- List of consistent indications
        self.symbols = list() # ------- Determinized beliefs
        self.facts   = list() # ------- Grounded predicates


    def reset_state( self ):
        """ Erase problem state """
        self.status = Status.INVALID # Running status
        self.task   = None # --------- Current task definition
        self.goal   = tuple() # ------ Current goal specification
        self.grasp  = list() # ------- ? NOT USED ?


    def __init__( self, world = None, noViz = False, noBot = False ):
        """ Create a pre-determined collection of poses and plan skeletons """
        # NOTE: The planner will start the Perception Process and the UR5 connection as soon as it is instantiated
        self.reset_beliefs()
        self.reset_state()
        self.world  = world if (world is not None) else VisualCortex( noViz = noViz )
        self.robot  = ur5.UR5_Interface() if (not noBot) else None
        self.logger = DataLogger() if (not noBot) else None
        self.noViz  = noViz
        self.noBot  = noBot
        # DEATH MONITOR
        self.noSoln =  0
        self.nonLim = 10
        if (not noBot):
            self.robot.start()


    def shutdown( self ):
        """ Stop the Perception Process and the UR5 connection """
        self.world.stop()
        if not self.noBot:
            self.robot.stop()


    def perceive_scene( self, xform = None ):
        """ Integrate one noisy scan into the current beliefs """
        scan = self.world.full_scan_noisy( xform )
        # LKG and Belief are updated SEPARATELY and merged LATER as symbols
        self.world.rectify_readings( copy_readings_as_LKG( scan ) )
        if _USE_GRAPHICS:
            display_belief_geo( self.world.get_last_best_readings() )
        self.memory.belief_update( scan, xform, maxRadius = _MAX_UPDATE_RAD_M )


    ##### Stream Helpers ##################################################

    def get_grounded_pose_or_new( self, homog ):
        """ If there is a `Waypoint` approx. to `homog`, then return it, Else create new `ObjPose` """
        for fact in self.facts:
            if fact[0] == 'Waypoint' and ( euclidean_distance_between_symbols( homog, fact[1] ) <= _ACCEPT_POSN_ERR):
                return fact[1]
        return ObjPose( homog )


    def p_grounded_fact_pose( self, poseOrObj ):
        """ Does this exist as a `Waypoint`? """
        homog = extract_pose_as_homog( poseOrObj )
        for fact in self.facts:
            if fact[0] == 'Waypoint' and ( euclidean_distance_between_symbols( homog, fact[1] ) <= _ACCEPT_POSN_ERR):
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
                    upPose = extract_pose_as_homog( sym )
                    upPose[2,3] += _BLOCK_SCALE

                    rtnPose = self.get_grounded_fact_pose_or_new( upPose )
                    print( f"FOUND a pose {rtnPose} supported by {objcName}!" )

                    yield (rtnPose,)

        return stream_func
    

    def get_placement_pose_stream( self ):
        """ Return a function that returns poses """

        def stream_func( *args ):
            """ A function that returns poses """

            if _VERBOSE:
                print( f"\nEvaluate PLACEMENT POSE stream with args: {args}\n" )

            placed   = False
            testPose = None
            while not placed:
                testPose  = rand_table_pose()
                print( f"\t\tSample: {testPose}" )
                for sym in self.symbols:
                    if euclidean_distance_between_symbols( testPose, sym ) < ( _MIN_SEP ):
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
            print( f"Symbols: {self.symbols}" )

            for sym in self.symbols:
                if euclidean_distance_between_symbols( chkPose, sym ) < ( _MIN_SEP ):
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
    

    ##### Object Permanence ###############################################

    def merge_and_reconcile_object_memories( self, tau = _SCORE_DECAY_TAU_S ):
        """ Calculate a consistent object state from LKG Memory and Beliefs """
        mrgLst  = list()
        tCurr   = now()
        totLst  = copy_readings( self.memory.beliefs[:] )
        LKGmem  = copy_readings( self.world.get_last_best_readings() )
        totLst.extend( LKGmem )
        
        # Filter and Decay stale readings
        for r in totLst:
            if ((tCurr - r.ts) <= _OBJ_TIMEOUT_S):
                score_r = np.exp( -(tCurr - r.ts) / tau ) * r.score
                if isnan( score_r ):
                    print( f"\nWARN: Got a NaN score with count {r.count}, distribution {r.labels}, and age {tCurr - r.ts}\n" )
                    score_r = 0.0
                r.score = score_r
                mrgLst.append( r )
        
        # Enforce consistency and return
        return self.world.rectify_readings( mrgLst, suppressStorage = True )
        

    def most_likely_objects( self, objList, method = "unique-non-null" ):
        """ Get the `N` most likely combinations of object classes """

        ### Combination Generator ###

        def gen_combos( objs ):
            ## Init ##
            comboList = [ [1.0,[],], ]
            ## Generate all class combinations with joint probabilities ##
            for bel in objs:
                nuCombos = []
                for combo_i in comboList:
                    for label_j, prob_j in bel.labels.items():
                        prob_ij = combo_i[0] * prob_j

                        objc_ij = GraspObj( label = label_j, pose = bel.pose, prob = prob_j, score = bel.score )
                        
                        nuCombos.append( [prob_ij, combo_i[1]+[objc_ij,],] )
                comboList = nuCombos
            ## Sort all class combinations with decreasing probabilities ##
            comboList.sort( key = (lambda x: x[0]), reverse = True )
            return comboList

        ### Filtering Methods ###

        def p_unique_labels( objs ):
            """ Return true if there are as many classes as there are objects """
            lbls = set([sym.label for sym in objs])
            return len( lbls ) == len( objs )
        
        def p_unique_non_null_labels( objs ):
            """ Return true if there are as many classes as there are objects """
            lbls = set([sym.label for sym in objs])
            if _NULL_NAME in lbls: 
                return False
            return len( lbls ) == len( objs )
        
        def clean_dupes_prob( objLst ):
            """ Return a version of `objLst` with duplicate objects removed """
            dctMax = {}
            for sym in objLst:
                if not sym.label in dctMax:
                    dctMax[ sym.label ] = sym
                elif sym.prob > dctMax[ sym.label ].prob:
                    dctMax[ sym.label ] = sym
            return list( dctMax.values() )
        
        def clean_dupes_score( objLst ):
            """ Return a version of `objLst` with duplicate objects removed """
            dctMax = {}
            for sym in objLst:
                if not sym.label in dctMax:
                    dctMax[ sym.label ] = sym
                elif sym.score > dctMax[ sym.label ].score:
                    dctMax[ sym.label ] = sym
            return list( dctMax.values() )

        ### Apply the chosen Filtering Method to all possible combinations ###

        totCombos  = gen_combos( objList )
        rtnSymbols = list()

        if (method == "unique"):
            for combo in totCombos:
                if p_unique_labels( combo[1] ):
                    rtnSymbols = combo[1]
                    break
        elif (method == "unique-non-null"):
            for combo in totCombos:
                if p_unique_non_null_labels( combo[1] ):
                    rtnSymbols = combo[1]
                    break
        elif (method == "clean-dupes"):
            rtnSymbols = clean_dupes_prob( totCombos[0][1] )
        elif (method == "clean-dupes-score"):
            rtnSymbols = clean_dupes_score( totCombos[0][1] )
        else:
            raise ValueError( f"`ResponsiveTaskPlanner.most_likely_objects`: Filtering method \"{method}\" is NOT recognized!" )
        
        ### Return all non-null symbols ###
        rtnLst = [sym for sym in rtnSymbols if sym.label != _NULL_NAME]
        print( f"\nDeterminized {len(rtnLst)} objects!\n" )
        return rtnLst
    
    



    ##### Noisy Task Monitoring ###########################################

    def get_sampled_block( self, label ):
        """ If a block with `label` was sampled, then return a reference to it, Otherwise return `None` """
        for sym in self.symbols:
            if sym.label == label:
                return sym
        return None
    

    def get_grounded_fact_pose_or_new( self, homog ):
        """ If there is a `Waypoint` approx. to `homog`, then return it, Else create new `ObjPose` """ 
        for fact in self.facts:
            if fact[0] == 'Waypoint' and (euclidean_distance_between_symbols( homog, fact[1] ) <= _ACCEPT_POSN_ERR):
                return fact[1]
            if fact[0] == 'GraspObj' and (euclidean_distance_between_symbols( homog, fact[2] ) <= _ACCEPT_POSN_ERR):
                return fact[2]
        return ObjPose( homog )
    
    
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
                if (tObj is not None) and (euclidean_distance_between_symbols( pPos, tObj ) <= _ACCEPT_POSN_ERR):
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
                    posUp = extract_pose_as_homog( sym_i )
                    posDn = extract_pose_as_homog( sym_j )
                    xySep = diff_norm( posUp[0:2,3], posDn[0:2,3] )
                    zSep  = posUp[2,3] - posDn[2,3] # Signed value
                    if ((xySep <= 1.65*_BLOCK_SCALE) and (1.65*_BLOCK_SCALE >= zSep >= 0.9*_BLOCK_SCALE)):
                        supDices.add(i)
                        rtnFacts.extend([
                            ('Supported', lblUp, lblDn,),
                            ('Blocked', lblDn,),
                            ('PoseAbove', self.get_grounded_fact_pose_or_new( posUp ), lblDn,),
                        ])
        for i, sym_i in enumerate( self.symbols ):
            if i not in supDices:
                rtnFacts.extend( [
                    ('Supported', sym_i.label, 'table',),
                    ('PoseAbove', self.get_grounded_fact_pose_or_new( sym_i ), 'table',),
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

        for _ in range( Nscans ):
            self.perceive_scene( xform ) # We need at least an initial set of beliefs in order to plan

        self.beliefs = self.merge_and_reconcile_object_memories()
        self.symbols = self.most_likely_objects( self.beliefs, method = "clean-dupes-score" ) # clean-dupes # clean-dupes-score # unique
        self.status  = Status.RUNNING

        if _VERBOSE:
            print( f"\nStarting Objects:" )
            for obj in self.symbols:
                print( f"\t{obj}" )
            if not len( self.symbols ):
                print( f"\tNO OBJECTS DETERMINIZED" )


    def allocate_table_swap_space( self, Nspots = _N_XTRA_SPOTS ):
        """ Find some open poses on the table for performing necessary swaps """
        rtnFacts  = []
        freeSpots = []
        occuSpots = [ extract_pose_as_homog( sym ) for sym in self.symbols]
        while len( freeSpots ) < Nspots:
            nuPose = rand_table_pose()
            print( f"\t\tSample: {nuPose}" )
            collide = False
            for spot in occuSpots:
                if euclidean_distance_between_symbols( spot, nuPose ) < ( _MIN_SEP ):
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
                    if abs( extract_pose_as_homog(g[2])[2,3] - _BLOCK_SCALE) < _ACCEPT_POSN_ERR:
                        self.facts.append( ('PoseAbove', g[2], 'table') )

            ## Ground the Blocks ##
            for sym in self.symbols:
                self.facts.append( ('Graspable', sym.label,) )

                blockPose = self.get_grounded_fact_pose_or_new( sym )

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
            self.action   = get_BT_plan_until_block_change( plan, self, _UPDATE_PERIOD_S )
            self.noSoln   = 0 # DEATH MONITOR
        else:
            self.noSoln += 1 # DEATH MONITOR
            self.logger.log_event( "NO SOLUTION" )
            self.status = Status.FAILURE


    def phase_4_Execute_Action( self ):
        """ Attempt to execute the first action in the symbolic plan """
        
        btr = BT_Runner( self.action, _BT_UPDATE_HZ, _BT_ACT_TIMEOUT_S )
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

            btr.per_sleep()

        self.logger.log_event( "BT END", str( btr.status ) )

        print( f"Did the BT move a reading?: {self.world.move_reading_from_BT_plan( self.action )}" )


    def phase_5_Return_Home( self, goPose ):
        """ Get ready for next iteration while updating beliefs """
        btAction = GroundedAction( args = list(), robot = self.robot, name = "Return Home" )
        btAction.add_children([
            Open_Gripper( ctrl = self.robot ),
            Interleaved_MoveFree_and_PerceiveScene( 
                MoveFree( [None, ObjPose( goPose )], robot = self.robot, suppressGrasp = True ), 
                self, 
                _UPDATE_PERIOD_S, 
                initSenseStep = True 
            ),
        ])
        
        btr = BT_Runner( btAction, _BT_UPDATE_HZ, _BT_ACT_TIMEOUT_S )
        btr.setup_BT_for_running()

        while not btr.p_ended():
            btr.tick_once()
            btr.per_sleep()

        print( f"\nRobot returned to \n{goPose}\n" )
        


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

    def p_belief_dist_OK( self ): 
        """ Return False if belief change criterion met, Otherwise return True """
        print( f"\nFIXME: `ResponsiveTaskPlanner.p_belief_dist_OK` HAS NOT BEEN IMPLEMENTED!!!\n", file = sys.stderr )
        return True


    def solve_task( self, maxIter = 100, beginPlanPose = None ):
        """ Solve the goal """
        
        if beginPlanPose is None:
            if _BLOCK_SCALE < 0.030:
                beginPlanPose = _GOOD_VIEW_POSE
            else:
                beginPlanPose = _HIGH_VIEW_POSE

        i = 0

        print( "\n\n\n##### TASK BEGIN #####\n" )

        self.reset_beliefs() 
        self.reset_state() 
        self.set_goal()
        self.logger.begin_trial()

        indicateSuccess = False
        t5              = now()

        self.robot.moveL( beginPlanPose, asynch = False ) # 2024-07-22: MUST WAIT FOR ROBOT TO MOVE

        while (self.status != Status.SUCCESS) and (i < maxIter): # and (not self.PANIC):

            
            # sleep(1)

            print( f"### Iteration {i+1} ###" )
            
            i += 1

            ##### Phase 1 ########################

            print( f"Phase 1, {self.status} ..." )

            self.set_goal()

            camPose = self.robot.get_cam_pose()

            expBgn = now()
            if (expBgn - t5) < _UPDATE_PERIOD_S:
                sleep( _UPDATE_PERIOD_S - (expBgn - t5) )
            
            self.phase_1_Perceive( 1, camPose )
            
            # if _USE_GRAPHICS:
            #     self.display_belief_geo()

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
            t4 = now()
            if (t4 - expBgn) < _UPDATE_PERIOD_S:
                sleep( _UPDATE_PERIOD_S - (t4 - expBgn) )
            self.phase_4_Execute_Action()

            ##### Phase 5 ########################

            print( f"Phase 5, {self.status} ..." )
            t5 = now()
            if (t5 - t4) < _UPDATE_PERIOD_S:
                sleep( _UPDATE_PERIOD_S - (t5 - t4) )
            self.phase_5_Return_Home( beginPlanPose )

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


########## CONFUSION MATRIX HELPERS ################################################################
from ast import literal_eval





class Confuser:
    """ Compute the confusion matrix """

    ##### Init ############################################################

    def open_file( self ):
        """ Set the name of the current file """
        dateStr     = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
        self.outNam = f"Confusion-Data_{dateStr}.txt"
        self.outFil = open( os.path.join( self.outDir, self.outNam ), 'w' )


    def __init__( self ):
        """ Set data collection params & robot connection """
        self.world  = VisualCortex( noViz = False ) 
        self.robot  = ur5.UR5_Interface() 
        self.outDir = "data/ConfMatx/"
        self.pausMv  = 1.0
        self.pausSh  = 3.0
        self.datLin = list()
        self.robot.start()
        self.open_file()


    def shutdown( self ):
        """ Stop the Perception Process and the UR5 connection """
        self.world.stop()
        self.robot.stop()


    ##### Data Collection #################################################

    def take_shot( self ):
        """ Get all `obs`ervations at a `cam`era pose """
        return {
            'obs' : self.world.get_perc_output(),
            'cam' : self.robot.get_cam_pose().tolist(),
        }
    
    def tcp_from_cam_pose( self, camPose ):
        return camPose.dot( np.linalg.inv( np.array( self.robot.camXform ) ) )
    
    def take_shot_at( self, desCamPose, capture = False, userCheck = True ):
        """ Place the camera at a given location and run perception pipeling """
        tcpPose = self.tcp_from_cam_pose( desCamPose )
        print( f"\n{tcpPose}\n" )
        if userCheck:
            ans = breakpoint( "Verify SANE pose!" )
        else:
            ans = 'y'
        if ('n' not in str( ans ).lower()):
            self.robot.moveL( tcpPose, asynch = False )
            sleep( self.pausMv )
            rtnDat = self.take_shot()
            sleep( self.pausSh )
            if capture:
                self.datLin.append( rtnDat )
            return rtnDat
        else:
            print( f"User REJECTED move to \n{tcpPose}" )
            return {'obs':None, 'cam':None,}


    def dump_to_file( self, openNext = False ):
        """ Write all data lines to a file """
        self.outFil.writelines( [f"{str(line)}\n" for line in self.datLin] )
        self.outFil.close()
        if openNext:
            self.datLin = list()
            self.open_file()


    def run_shot_program( self, program, actual = None, openNext = False, userCheck = True ):
        """ Go to every camera pose in the `program` and take a shot, Dump data when complete """
        print( "\n##### Run Program #####\n" )
        outN = self.outNam
        if actual is not None:
            self.datLin.append( [[pair[0],pair[1].tolist()] for pair in actual] )
        for i, pose in enumerate( program ):
            print( f"\n## Pose {i+1} ##" )
            self.take_shot_at( pose, capture = True, userCheck = userCheck )
        self.dump_to_file( openNext )
        print( f"\n##### Program COMPLETE --wrote-> {self.outDir}{outN} #####\n" )

    

########## DATA PROCESSING #########################################################################

def read_program_output( inPath, actual = False ):
    """ Read all data lines written by `dump_to_file` """
    rtnData = []
    with open( inPath, 'r' ) as f:
        lines = f.readlines()
        if actual:
            pairs  = literal_eval( lines[0] )
            rtnAct = list()
            for name, pose in pairs:
                rtnAct.append([
                    name, 
                    np.array( pose )
                ])
            alines = lines[1:]
        else:
            alines = lines
        for line in alines:
            if len( line ) > 5:
                shot   = dict()
                struct = literal_eval( line )
                shot['cam'] = np.array( struct['cam'] ).reshape( (4,4,) )
                shot['obs'] = list()
                for obs_i in struct['obs'].values():
                    pose_i = shot['cam'].dot( np.array( obs_i['Pose'] ).reshape( (4,4,) ) )
                    pose_i[0:3,0:3] = np.eye(3)
                    shot['obs'].append({
                        'Probability' : obs_i[ 'Probability' ],
                        'Count'       : obs_i[ 'Count'       ],
                        'Time'        : obs_i[ 'Time'        ],
                        'Pose'        : pose_i                ,
                    })
                rtnData.append( shot )
    if actual:
        return rtnData, rtnAct
    else:
        return rtnData
        


########## DATA GATHERING MOTION PLANS #############################################################


def camera_pose_from_target_offset( trgtPose, offsetVec ):
    """ Get a "fingers flat" camera pose with the wrist kinda pointed towards the chair """
    zBasis  = -vec_unit( offsetVec )
    yBasis  =  vec_unit( np.cross( zBasis, [0.0, 0.0, 1.0,] ) )
    xBasis  = np.cross( yBasis, zBasis )
    positn  = posn_from_xform( trgtPose ) + np.array( offsetVec )
    rtnPose = np.eye(4)
    rtnPose[0:3,0] = xBasis
    rtnPose[0:3,1] = yBasis
    rtnPose[0:3,2] = zBasis
    rtnPose[0:3,3] = positn
    return repair_pose( rtnPose )


def make_program_poses( bgnTrgtPose, offsetVec, moveDir, dStep_m, Nstep ):
    """ Build a pose program using `camera_pose_from_target_offset` """
    moveDir = vec_unit( moveDir )
    stepVec = moveDir * dStep_m
    bgnPose = camera_pose_from_target_offset( bgnTrgtPose, offsetVec )
    rtnProg = [bgnPose,]
    for _ in range( 1, Nstep ):
        pose_i = rtnProg[-1].copy()
        pose_i[0:3,3] += stepVec
        rtnProg.append( pose_i )
    return rtnProg
        

_HS             = _BLOCK_SCALE/2.0
_BLOCK_RAD_M    = np.sqrt( 3.0 * _HS**2 )
_CONF_BLC_POSES = [
    np.array([[ 1.,     0.,     0.,    -0.286,],
              [ 0.,     1.,     0.,    -0.291,],
              [ 0.,     0.,     1.,     _HS  ,],
              [ 0.,     0.,     0.,     1.   ,],]),

    np.array([[ 1.,     0.,     0.,    -0.297,],
              [ 0.,     1.,     0.,    -0.413,],
              [ 0.,     0.,     1.,     _HS  ,],
              [ 0.,     0.,     0.,     1.   ,],]),

    np.array([[ 1.,     0.,     0.,    -0.304,],
              [ 0.,     1.,     0.,    -0.537,],
              [ 0.,     0.,     1.,     _HS  ,],
              [ 0.,     0.,     0.,     1.   ,],]),
]


    

########## MAIN ####################################################################################
_CONF_MATRIX    = 1
_TROUBLESHOOT   = 0
_VISION_TEST    = 0
_EXP_BGN_POSE   = _HIGH_VIEW_POSE




_CONF_CAM_POSE_ANGLED1 = repair_pose( np.array( [[ 0.55 , -0.479,  0.684, -0.45 ],
                                                 [-0.297, -0.878, -0.376, -0.138],
                                                 [ 0.781,  0.003, -0.625,  0.206],
                                                 [ 0.   ,  0.   ,  0.   ,  1.   ],] ) )


if __name__ == "__main__":

    dateStr = datetime.now().strftime("%m/%d/%Y, %H:%M:%S")

    if _CONF_MATRIX:

        if 1:

            bgnTrgt = _CONF_BLC_POSES[0].copy()
            bgnTrgt[1,3] += 0.075

            prog = list()

            for dist in [0.200, 0.300, 0.400]:
                prog.extend(
                    make_program_poses( bgnTrgt, vec_unit( [-0.200,0.200,dist] )*(dist+_BLOCK_RAD_M), [0,-1,0], 0.050, 11 )
                )

            # actBlc = list( zip(['ylw','blu','NOT'], _CONF_BLC_POSES) ) # Config 1
            # actBlc = list( zip(['blu','grn','NOT'], _CONF_BLC_POSES) ) # Config 2
            # actBlc = list( zip(['ylw','grn','NOT'], _CONF_BLC_POSES) ) # Config 3
            # actBlc = list( zip(['blu','ylw','NOT'], _CONF_BLC_POSES) ) # Config 4
            # actBlc = list( zip(['grn','blu','NOT'], _CONF_BLC_POSES) ) # Config 5
            actBlc = list( zip(['grn','ylw','NOT'], _CONF_BLC_POSES) ) # Config 6

            # camTst = camera_pose_from_target_offset( _CONF_BLC_POSES[0], vec_unit( [-1,1,1] )*0.200 )
            # print( camTst )

            ctrl = Confuser()
            ctrl.run_shot_program( prog, actual = actBlc, openNext = False, userCheck = False )
            ctrl.shutdown()

            # tcpCom = repair_pose( camAct.dot( np.transpose( np.array( rbt.camXform ) ) ) )
            # tcpCom = camAct.dot( np.linalg.inv( np.array( rbt.camXform ) ) ) 
            # print( tcpAct )
            # print( tcpCom )
            
            # print( f"Computed Difference: {np.linalg.norm( tcpAct - tcpCom )}" )
            # rbt.stop()

        elif 0:

            # prog = [ _CONF_CAM_POSE_ANGLED1.copy(), ]
            # for _ in range( int(40/5) ):
            #     pose_i = prog[-1].copy()
            #     pose_i[1,3] -= 0.050
            #     prog.append( pose_i )
            
            # ctrl = Confuser()
            # ctrl.run_shot_program( prog, openNext = False, userCheck = True )
            # ctrl.shutdown()

            poseData = read_program_output( "data/ConfMatx/Confusion-Data_07-29-2024_17-14-35.txt" )
            cnfPoses = []

            _ACCEPT_RAD = 1.5*_BLOCK_SCALE

            for shot in poseData:
                for obs in shot['obs']:
                    pose_j = obs['Pose']
                    found  = False
                    for pose_k in cnfPoses:
                        if translation_diff( pose_j, pose_k ) <= _ACCEPT_RAD:
                            found = True
                            pose_k[0:3,3] = (posn_from_xform( pose_j ) + posn_from_xform( pose_k ))/2.0
                            break
                    if (not found) and p_symbol_inside_workspace_bounds( pose_j ):
                        cnfPoses.append( pose_j )
            for pose_k in cnfPoses:
                print( pose_k )
                print()
            

    elif _TROUBLESHOOT:
        print( f"########## Running Debug Code at {dateStr} ##########" )
        from graphics.homog_utils import R_x, homog_xform


        planner = ResponsiveTaskPlanner( noViz = True, noBot = True )
        blcPosn = {
            "good": [ 0.0  ,  0.0  ,  0.140,],
            "bad1": [ 0.0  ,  0.140,  0.0  ,],
            "bad2": [ 0.140,  0.0  ,  0.0  ,],
            "bad3": [ 0.0  , -0.140,  0.0  ,],
            "bad4": [-0.140,  0.0  ,  0.0  ,],
            "bad5": [ 0.0  ,  0.0  , -0.140,],

        }
        blcPose = np.eye(4)
        camPose = np.eye(4)
        camPose = camPose.dot( homog_xform( R_x(np.pi/2.0), [0,0,0] ) )

        for k, v in blcPosn.items():
            blcPose[0:3,3] = v
            print( f"Pose: {k}, Passed?: {planner.memory.p_symbol_in_cam_view( camPose, blcPose )}\n" )

        
        if 0:
            rbt = ur5.UR5_Interface()
            rbt.start()
            rbt.moveL( repair_pose( _GOOD_VIEW_POSE ), asynch = False )
            sleep(5)
            rbt.moveL( repair_pose( _HIGH_VIEW_POSE ), asynch = False )
            # print( f"Began at pose:\n{rbt.get_tcp_pose()}" )
            sleep(1)
            rbt.stop()


    elif _VISION_TEST:
        print( f"########## Running Vision Pipeline Test at {dateStr} ##########" )

        planner = responsive_experiment_prep( _HIGH_VIEW_POSE )
        
        print( f"\nAt Pose 1:\n{_HIGH_VIEW_POSE}\n" )
        planner.world.perc.capture_image()
        sleep(1)
        
        planner.robot.moveL( _HIGH_TWO_POSE, asynch = False )
        print( f"\nAt Pose 2:\n{_HIGH_TWO_POSE}\n" )
        planner.world.perc.capture_image()
        sleep(1)

        observs = planner.world.perc.merge_and_build_model()
        xfrmCam = planner.robot.get_cam_pose()
        planner.world.full_scan_noisy( xfrmCam, observations = observs )
        # if _USE_GRAPHICS:
        #     planner.memory.display_belief_geo( planner.world.scan )

        planner.world.rectify_readings( copy_readings_as_LKG( planner.world.scan ) )
        observs = planner.world.get_last_best_readings()
        planner.world.full_scan_noisy( xfrmCam, observations = observs )
        # if _USE_GRAPHICS:
        #     planner.memory.display_belief_geo( planner.world.scan )

        sleep( 2.5 )
        planner.shutdown()


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
            print_exc()
            print()
            planner.shutdown()

        except Exception as e:
            # Bad Thing: Attempt to shut down gracefully
            print( f"Something BAD happened!: {e}" )
            print_exc()
            print()
            planner.shutdown()

    os.system( 'kill %d' % os.getpid() ) 

    
        