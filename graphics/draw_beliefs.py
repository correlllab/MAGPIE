########## INIT ####################################################################################

import sys

import numpy as np
import open3d as o3d

### Local ###
from dh_mp import UR5_DH
from dh_graphics import plot_DH_robot
sys.path.append( "../" )
from env_config import ( _MIN_X_OFFSET, _MIN_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN )

_TABLE_THIC = 0.030



########## DRAWING FUNCTIONS #######################################################################

def table_and_origin_geo( axesScale = 0.050 ):
    """ Draw the usable workspace """
    rtnLst = list()
    table  = o3d.geometry.TriangleMesh.create_box( _X_WRK_SPAN, _Y_WRK_SPAN, _TABLE_THIC )
    table.transform( np.array(
        [ [1.0, 0.0, 0.0,  _MIN_X_OFFSET ] ,
          [0.0, 1.0, 0.0,  _MIN_Y_OFFSET ] ,
          [0.0, 0.0, 1.0, -_TABLE_THIC/2.0 ] ,
          [0.0, 0.0, 0.0,  1.0   ] ] 
    ) )
    table.paint_uniform_color( [237/255.0, 139/255.0, 47/255.0] ) # Plywood/Sand color
    rtnLst.append( table )
    rtnLst.append( o3d.geometry.TriangleMesh.create_coordinate_frame( size = axesScale ) )
    return rtnLst


def UR5_geo( qConfig ):
    """ Compute drawable geo of the UR5 in `qConfig` """
    return plot_DH_robot( UR5_DH, qConfig, getDrawList=1, suppressTable=1 )


def belief_geo( beliefs ):
    """ Get drawable geometry for the given `beliefs` """
    pass