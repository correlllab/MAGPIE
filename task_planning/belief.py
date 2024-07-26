########## INIT ####################################################################################

###### Imports ######

### Standard ###
import atexit, sys
from time import sleep

### Special ###
import numpy as np
import open3d as o3d
from pyglet.window import Window

### Local ###
from env_config import ( _BLOCK_SCALE, _N_CLASSES, _CONFUSE_PROB, _NULL_NAME, _NULL_THRESH, 
                         _BLOCK_NAMES, _VERBOSE, _SCORE_FILTER_EXP, )
from task_planning.utils import ( extract_dct_values_in_order, sorted_obj_labels, multiclass_Bayesian_belief_update, get_confusion_matx, 
                                  get_confused_class_reading )
sys.path.append( "../magpie/" )
from magpie.poses import translation_diff

########## HELPER FUNCTIONS ########################################################################

def d_between_obj_poses( obj1, obj2 ):
    """ Calculate the translation between poses in the same frame """
    return np.linalg.norm( np.subtract( obj1.pose[:3], obj2.pose[:3] ) )


def extract_class_dist_in_order( obj, order = _BLOCK_NAMES ):
    """ Get the discrete class distribution, in order according to environment variable """
    if isinstance( obj, dict ):
        return np.array( extract_dct_values_in_order( obj, order ) )
    else:
        return np.array( extract_dct_values_in_order( obj.labels, order ) )


def extract_class_dist_sorted_by_key( obj ):
    """ Get the discrete class distribution, sorted by key name """
    return np.array( extract_dct_values_in_order( obj.labels, sorted_obj_labels( obj ) ) )


def extract_class_dist_in_order( obj, order = _BLOCK_NAMES ):
    """ Get the discrete class distribution, in order according to environment variable """
    return np.array( extract_dct_values_in_order( obj.labels, order ) )

def vis_window( geo ):
    """ Open3D SUUUUUUUUUCKS """
    # https://github.com/isl-org/Open3D/issues/3489#issuecomment-863704146
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry( geo )
    ctr = vis.get_view_control()  # Everything good
    print( f"Opened window: {ctr}" )
    vis.run()


def exp_filter( lastVal, nextVal, rate01 ):
    """ Blend `lastVal` with `nextVal` at the specified `rate` (Exponential Filter) """
    assert 0.0 <= rate01 <= 1.0, f"Exponential filter rate must be on [0,1], Got {rate01}"
    return (nextVal * rate01) + (lastVal * (1.0 - rate01))



########## BELIEFS #################################################################################


class ObjectMemory:
    """ Attempt to maintain recent and constistent object beliefs based on readings from the vision system """

    def reset_beliefs( self ):
        """ Remove all references to the beliefs, then erase the beliefs """
        self.beliefs = []
        # self.vis.clear_geometries()


    def __init__( self ):
        """ Set belief containers """
        # https://github.com/isl-org/Open3D/issues/2006#issuecomment-701340077
        # self.vis = o3d.visualization.Visualizer()
        # self.vis.create_window( window_name = "Planner: Current Belief", width = 100, height = 700, visible = True )
        self.reset_beliefs()
        # atexit.register( self.destroy )


    # def destroy( self ):
    #     """ Close window and cleanup """
    #     self.vis.destroy_window()


    

        # vis_window( geo )
        
    
    def accum_evidence_for_belief( self, evidence, belief ):
        """ Use Bayesian multiclass update on `belief`, destructive """
        evdnc = extract_class_dist_in_order( evidence )
        prior = extract_class_dist_in_order( belief   )
        keys  = _BLOCK_NAMES
        pstrr = multiclass_Bayesian_belief_update( 
            get_confusion_matx( _N_CLASSES, confuseProb = _CONFUSE_PROB ), 
            prior, 
            evdnc 
        )
        for i, key in enumerate( keys ):
            belief.labels[ key ] = pstrr[i]


    def integrate_one_reading( self, objReading, maxRadius = 3.0*_BLOCK_SCALE ):
        """ Fuse this belief with the current beliefs """
        relevant = False

        # 1. Determine if this belief provides evidence for an existing belief
        dMin     = 1e6
        belBest  = None
        for belief in self.beliefs:
            d = d_between_obj_poses( objReading, belief )
            if (d < maxRadius) and (d < dMin):
                dMin     = d
                belBest  = belief
                relevant = True

        if relevant:
            belBest.visited = True
            self.accum_evidence_for_belief( objReading, belBest )
            # belBest.pose = np.array( objReading.pose ) # WARNING: ASSUME THE NEW NEAREST POSE IS CORRECT!
            belBest.pose = objReading.pose # WARNING: ASSUME THE NEW NEAREST POSE IS CORRECT!
            belBest.score = exp_filter( belBest.score, objReading.score, _SCORE_FILTER_EXP )

        # 2. If this evidence does not support an existing belief, it is a new belief
        else:
            self.beliefs.append( objReading.copy() ) 

        # N. Return whether the reading was relevant to an existing belief
        return relevant
    

    def integrate_null( self, belief, avgScore = None ):
        """ Accrue a non-observation """
        labels = get_confused_class_reading( _NULL_NAME, _CONFUSE_PROB, _BLOCK_NAMES )
        cnfMtx = get_confusion_matx( _N_CLASSES, _CONFUSE_PROB )
        priorB = [ belief.labels[ label ] for label in _BLOCK_NAMES ] 
        evidnc = [ labels[ label ] for label in _BLOCK_NAMES ]
        updatB = multiclass_Bayesian_belief_update( cnfMtx, priorB, evidnc )
        belief.labels = {}
        for i, name in enumerate( _BLOCK_NAMES ):
            belief.labels[ name ] = updatB[i]
        if avgScore is not None:
            belief.score = exp_filter( belief.score, avgScore, _SCORE_FILTER_EXP )
    

    def unvisit_beliefs( self ):
        """ Set visited flag to False for all beliefs """
        for belief in self.beliefs:
            belief.visited = False


    def erase_dead( self ):
        """ Erase all beliefs and cached symbols that no longer have relevancy """
        retain = []
        for belief in self.beliefs:
            if belief.labels[ _NULL_NAME ] < _NULL_THRESH:
                retain.append( belief )
            elif _VERBOSE:
                print( f"{str(belief)} DESTROYED!" )
        self.beliefs = retain


    def decay_beliefs( self ):
        """ Destroy beliefs that have accumulated too many negative indications """
        vstScores = list()
        for belief in self.beliefs:
            if belief.visited:
                vstScores.append( belief.score )
        nuScore = np.mean( vstScores )
        for belief in self.beliefs:
            if (not belief.visited):
                self.integrate_null( belief, avgScore = nuScore )
        self.erase_dead()
        self.unvisit_beliefs()


    def belief_update( self, evdncLst ):
        """ Gather and aggregate evidence """

        ## Integrate Beliefs ##
        cNu = 0
        cIn = 0
        self.unvisit_beliefs()
        
        if not len( self.beliefs ):
            # WARNING: ASSUMING EACH OBJECT IS REPRESENTED BY EXACTLY 1 READING
            for objEv in evdncLst:
                self.beliefs.append( objEv.copy() )
        else:
            for objEv in evdncLst:
                # if not objEv.visitRD:
                #     objEv.visitRD = True
                if self.integrate_one_reading( objEv ):
                    cIn += 1
                else:
                    cNu += 1
            ## Decay Irrelevant Beliefs ##
            self.decay_beliefs()

        if _VERBOSE:
            if (cNu or cIn):
                print( f"\t{cNu} new object beliefs this iteration!" )
                print( f"\t{cIn} object beliefs updated!" )
            else:
                print( f"\tNO belief update!" )
        
        if _VERBOSE:
            print( f"Total Beliefs: {len(self.beliefs)}" )
            for bel in self.beliefs:
                print( f"\t{bel}" )
            print()


    
