########## INIT ####################################################################################
from copy import deepcopy
from itertools import count

import numpy as np

from utils import NaN_row_vec
from env_config import _NULL_NAME



########## HELPER FUNCTIONS ########################################################################

def extract_row_vec_pose( obj_or_arr ):
    """ Return only a copy of the row vector representing the 3D pose """
    if isinstance( obj_or_arr, (list,np.ndarray,) ):
        return np.array( obj_or_arr )
    elif isinstance( obj_or_arr, ObjPose ):
        return np.array( obj_or_arr.pose )
    elif isinstance( obj_or_arr, GraspObj ):
        return np.array( obj_or_arr.pose.pose )
    else:
        return NaN_row_vec()



########## COMPONENTS ##############################################################################


class ObjPose:
    """ Combination of Position and Orientation (Quat) with a Unique Index """
    num = count()

    def __init__( self, pose = None ):
        self.pose  = np.array( pose ) if isinstance(pose, (list,np.ndarray,)) else np.array([0,0,0,1,0,0,0])
        self.index = next( self.num )

    def row_vec( self ):
        """ Return the vector value of the pose """
        return np.array( self.pose )
    
    def copy( self ):
        """ Return a copy of this pose with a new index """
        return ObjPose( self.pose )
    
    def __repr__( self ):
        """ Text representation """
        return f"<ObjPose {self.index}, Vec: {self.pose} >"
    


class GraspObj:
    """ The concept of a named object at a pose """
    num = count()

    def __init__( self, label = None, pose = None ):
        self.label = label if (label is not None) else _NULL_NAME
        # self.pose  = pose if isinstance(pose, (list,np.ndarray)) else np.array([0,0,0,1,0,0,0])
        self.pose  = pose if (pose is not None) else ObjPose( [0,0,0,1,0,0,0] )
        self.index = next( self.num )


    def __repr__( self ):
        """ Text representation of noisy reading """
        return f"<GraspObj {self.index} @ {self.pose}, Class: {str(self.label)}>"



class ObjectReading( GraspObj ):
    """ Represents a signal coming from the vision pipeline """

    def __init__( self, labels = None, pose = None ):
        """ Init distribution and  """
        super().__init__( None, pose )
        self.labels  = labels if (labels is not None) else {} # Current belief in each class
        self.visited = False


    def __repr__( self ):
        """ Text representation of noisy reading """
        return f"<ObjectReading @ {self.pose}, Dist: {str(self.labels)}>"
    

    def copy( self ):
        """ Make a copy of this belief """
        # return ObjectReading( deepcopy( self.labels ), self.pose.copy() )
        return ObjectReading( deepcopy( self.labels ), self.pose )
