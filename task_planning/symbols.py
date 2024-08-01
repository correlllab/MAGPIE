########## INIT ####################################################################################

##### Imports #####
### Standard ###
import time
now = time.time
from copy import deepcopy
from itertools import count

### Special ###
import numpy as np

### Local ###
from graphics.homog_utils import R_quat, homog_xform
from magpie.poses import translation_diff
from env_config import ( _NULL_NAME, _NULL_NAME, 
                         _MIN_X_OFFSET, _MAX_X_OFFSET, _MIN_Y_OFFSET, _MAX_Y_OFFSET, _MAX_Z_BOUND, )



########## HELPER FUNCTIONS ########################################################################

def extract_np_array_pose( obj_or_arr ):
    """ Return only a copy of the row vector representing the 3D pose """
    if isinstance( obj_or_arr, (list, np.ndarray,) ):
        return np.array( obj_or_arr )
    elif isinstance( obj_or_arr, ObjPose ):
        return np.array( obj_or_arr.pose )
    elif isinstance( obj_or_arr, (GraspObj, ObjectReading,) ):
        return np.array( obj_or_arr.pose.pose )
    else:
        return None
    

def extract_pose_as_homog( obj_or_arr, noRot = False ):
    """ Return only a copy of the homogeneous coordinates of the 3D pose """
    bgnPose = extract_np_array_pose( obj_or_arr )
    if len( bgnPose ) == 4:
        rtnPose = bgnPose
    elif len( bgnPose ) == 16:
        rtnPose = np.array( bgnPose ).reshape( (4,4,) )
    else:
        raise ValueError( f"`extract_pose_as_homog`: Poorly formed pose:\n{bgnPose}" )
    if noRot:
        rtnPose[0:3,0:3] = np.eye(3)
    return rtnPose
    

def extract_position( obj_or_arr ):
    """ Return only a copy of the position vector of the 3D pose """
    pose = extract_pose_as_homog( obj_or_arr )
    return pose[0:3,3]


def p_symbol_inside_workspace_bounds( obj_or_arr ):
    """ Return True if inside the bounding box, Otherwise return False """
    posn = extract_position( obj_or_arr )      
    return (_MIN_X_OFFSET <= posn[0] <= _MAX_X_OFFSET) and (_MIN_Y_OFFSET <= posn[1] <= _MAX_Y_OFFSET) and (0.0 < posn[2] <= _MAX_Z_BOUND)


def euclidean_distance_between_symbols( sym1, sym2 ):
    """ Extract pose component from symbols and Return the linear distance between those poses """
    pose1 = extract_pose_as_homog( sym1 )
    pose2 = extract_pose_as_homog( sym2 )
    return translation_diff( pose1, pose2 )


########## COMPONENTS ##############################################################################


class ObjPose:
    """ Combination of Position and Orientation (Quat) with a Unique Index """
    num = count()

    def __init__( self, pose = None ):
        self.pose  = extract_pose_as_homog( pose ) if (pose is not None) else np.eye(4)
        self.index = next( self.num )

    def row_vec( self ):
        """ Return the vector value of the pose """
        return np.array( self.pose )
    
    def copy( self ):
        """ Return a copy of this pose with a new index """
        return ObjPose( self.pose )
    
    def __repr__( self ):
        """ Text representation """
        return f"<ObjPose {self.index}, Vec: {extract_position( self.pose )} >"
    


class GraspObj:
    """ The concept of a named object at a pose """
    # FIXME: JUST MERGE THIS WITH `ObjectReading` THEY ARE THE SAME NOW
    num = count()

    def __init__( self, label = None, pose = None, prob = None, score = None, labels = None ):
        """ Set components used by planners """
        self.label  = label if (label is not None) else _NULL_NAME
        self.labels = labels if (labels is not None) else dict()
        self.pose   = pose if (pose is not None) else ObjPose( np.eye(4) )
        self.index  = next( self.num )
        self.prob   = prob if (prob is not None) else 0.0 # 2024-07-22: This is for sorting dupes in the planner and is NOT used by PDDLStream
        self.score  = score if (score is not None) else 0.0 # 2024-07-25: This is for sorting dupes in the planner and is NOT used by PDDLStream


    def __repr__( self ):
        """ Text representation of noisy reading """
        return f"<GraspObj {self.index} @ {extract_position( self.pose )}, Class: {str(self.label)}, Score: {str(self.score)}>"
    
    def get_dict( self ):
        """ Return a verison of the `GraspObj` suitable for a TXT file """
        return {
            'name'  : self.__class__.__name__,
            'time'  : now(),
            'label' : self.label,
            'labels': self.labels,
            'pose'  : extract_pose_as_homog( self.pose ).tolist(),
            'index' : self.index,
            'prob'  : self.prob,
            'score' : self.score,
        }



class ObjectReading( GraspObj ):
    """ Represents a signal coming from the vision pipeline """

    def __init__( self, labels = None, pose = None, ts = None, count = 0, score = 0.0 ):
        """ Init distribution and  """
        super().__init__( None, pose )
        self.labels  = labels if (labels is not None) else {} # Current belief in each class
        self.visited = False # -------------------------------- Flag: Was this belief associated with a relevant reading
        self.ts      = ts if (ts is not None) else now() # ---- When was this reading created? [epoch time]
        self.count   = count # -------------------------------- How many bounding boxes generated this reading?
        self.score   = score # -------------------------------- Quality rating for this information
        self.LKG     = False # -------------------------------- Flag: Part of the Last-Known-Good collection?


    def __repr__( self ):
        """ Text representation of noisy reading """
        cleanDict = {}
        objAge    = now() - self.ts
        for k, v in self.labels.items():
            cleanDict[ k[:3] ] = np.round( v, 3 )
        return f"<ObjectReading @ {extract_position( self.pose )}, Dist: {str(cleanDict)}, Score: {self.score:.4f}, Age: {objAge:.2f} >"
    
    
    def get_dict( self ):
        """ Return a verison of the `GraspObj` suitable for a TXT file """
        return {
            'name'  : self.__class__.__name__,
            'time'  : now(),
            'ts'    : self.ts,
            'labels': self.labels,
            'pose'  : extract_pose_as_homog( self.pose ).tolist(),
            'index' : self.index,
            'score' : self.score,
        }

    def copy( self ):
        """ Make a copy of this belief """
        rtnObj = ObjectReading()
        rtnObj.labels  = deepcopy( self.labels ) # Current belief in each class
        rtnObj.pose    = self.pose
        rtnObj.visited = False # ----------------- Flag: Was this belief associated with a relevant reading
        rtnObj.ts      = self.ts # --------------- When was this reading created? [epoch time]
        rtnObj.count   = self.count # ------------ How many bounding boxes generated this reading?
        rtnObj.score   = self.score # ------------ Quality rating for this information
        rtnObj.LKG     = False # ----------------- Flag: Part of the Last-Known-Good collection?
        return rtnObj
    
    
    def copy_as_LKG( self ):
        """ Make a copy of this belief for the Last-Known-Good collection """
        rtnObj = self.copy()
        rtnObj.LKG = True
        return rtnObj
