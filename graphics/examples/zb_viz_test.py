import sys

import numpy as np
import open3d as o3d

### Local ###
from dh_mp import UR5_DH, UR5_COM, UR5_mass, robot_COM_for_q
from dh_graphics import plot_DH_robot
sys.path.append( "../" )
from env_config import ( _MIN_X_OFFSET, _MAX_X_OFFSET, _MIN_Y_OFFSET, _MAX_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN )


print( _MAX_X_OFFSET, _MIN_X_OFFSET, _X_WRK_SPAN, '\n', _MAX_Y_OFFSET, _MIN_Y_OFFSET, _Y_WRK_SPAN )
_TABLE_THIC = 0.030

def table_and_origin_geo( axesScale = 0.050 ):
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

geo = table_and_origin_geo( axesScale = 0.050 )
geo.extend( plot_DH_robot( UR5_DH, [0,0,0,0,0,0,], getDrawList=1, suppressTable=1 ) )

o3d.visualization.draw_geometries( geo )