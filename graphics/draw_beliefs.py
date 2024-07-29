########## INIT ####################################################################################

import sys

import numpy as np
import open3d as o3d

### Local ###
from graphics.dh_mp import UR5_DH
from graphics.dh_graphics import plot_DH_robot
from graphics.homog_utils import homog_xform, R_quat
sys.path.append( "../" )
from env_config import ( _MIN_X_OFFSET, _MIN_Y_OFFSET, _X_WRK_SPAN, _Y_WRK_SPAN, _BLOCK_SCALE, _CLR_TABLE, )
from task_planning.symbols import extract_pose_as_homog

_TABLE_THIC = 0.015


########## HELPER FUNCTIONS ########################################################################

def zip_dict_sorted_by_decreasing_value( dct ):
    """ Return a list of (k,v) tuples sorted by decreasing value """
    keys = list()
    vals = list()
    for k, v in dct.items():
        keys.append(k)
        vals.append(v)
    return sorted( zip( keys, vals ), key=lambda x: x[1], reverse=1)


########## DRAWING CLASS #########################################################################

class BeliefScene:
    pass

########## DRAWING FUNCTIONS #######################################################################

def table_and_origin_geo( axesScale = 0.050 ):
    """ Draw the usable workspace """
    rtnLst = list()
    table  = o3d.geometry.TriangleMesh.create_box( _X_WRK_SPAN, _Y_WRK_SPAN, _TABLE_THIC )
    table.transform( np.array(
        [ [1.0, 0.0, 0.0,  _MIN_X_OFFSET ] ,
          [0.0, 1.0, 0.0,  _MIN_Y_OFFSET ] ,
          [0.0, 0.0, 1.0, -_TABLE_THIC   ] ,
          [0.0, 0.0, 0.0,  1.0   ] ] 
    ) )
    table.paint_uniform_color( [237/255.0, 139/255.0, 47/255.0] ) # Plywood/Sand color
    rtnLst.append( table )
    rtnLst.append( o3d.geometry.TriangleMesh.create_coordinate_frame( size = axesScale ) )
    return rtnLst


def UR5_geo( qConfig ):
    """ Compute drawable geo of the UR5 in `qConfig` """
    return plot_DH_robot( UR5_DH, qConfig, getDrawList=1, suppressTable=1 )


def wireframe_box_geo( xScl, yScl, zScl ):
    """ Draw a wireframe cuboid """
    xHf = xScl/2.0
    yHf = yScl/2.0
    zHf = zScl/2.0
    verts = np.array([
        [ -xHf, -yHf, +zHf ],
        [ +xHf, -yHf, +zHf ],
        [ +xHf, +yHf, +zHf ],
        [ -xHf, +yHf, +zHf ],
        [ -xHf, -yHf, -zHf ],
        [ +xHf, -yHf, -zHf ],
        [ +xHf, +yHf, -zHf ],
        [ -xHf, +yHf, -zHf ],
    ])
    ndces = np.array([
        [0,4,],
        [1,5,],
        [2,6,],
        [3,7,],
        [0,1,],
        [1,2,],
        [2,3,],
        [3,0,],
        [4,5,],
        [5,6,],
        [6,7,],
        [7,4,],
    ])
    wireBox = o3d.geometry.LineSet( 
        o3d.utility.Vector3dVector( np.array( verts ) ), # Vertices
        o3d.utility.Vector2iVector( np.array( ndces ) ) #- Indices
    )
    wireBox.paint_uniform_color( [0/255.0, 0/255.0, 0/255.0] )
    return wireBox


def reading_geo( objReading ):
    """ Get geo for a single observation """
    labelSort = zip_dict_sorted_by_decreasing_value( objReading.labels )
    objXfrm   = extract_pose_as_homog( objReading, noRot = True )
    print( objXfrm )
    hf        = _BLOCK_SCALE/2.0
    topCrnrs  = [
        homog_xform( np.eye(3), [-hf+hf,-hf+hf, _BLOCK_SCALE,] ),
        homog_xform( np.eye(3), [-hf+hf, hf+hf, _BLOCK_SCALE,] ),
        homog_xform( np.eye(3), [ hf+hf,-hf+hf, _BLOCK_SCALE,] ),
        homog_xform( np.eye(3), [ hf+hf, hf+hf, _BLOCK_SCALE,] ),
    ]
    
    rtnGeo  = list()
    
    wir = wireframe_box_geo( _BLOCK_SCALE, _BLOCK_SCALE, _BLOCK_SCALE )
    wir.transform( objXfrm )
    rtnGeo.extend( [wir,] )
    
    for i in range(3):
        objXfrm[i,3] -= hf
    for i in range( 1, min( len(labelSort), len(topCrnrs) ) ):
        prob_i = labelSort[i][1]
        if (prob_i > 0.0):
            scal_i  = _BLOCK_SCALE * prob_i
            xfrm_i = topCrnrs[i-1]
            for j in range(3):
                xfrm_i[j,3] -= scal_i/2.0
            xfrm_i = objXfrm.dot( xfrm_i )
            bloc_i = o3d.geometry.TriangleMesh.create_box( scal_i, scal_i, scal_i )
            bloc_i.transform( xfrm_i )
            bloc_i.paint_uniform_color( _CLR_TABLE[ labelSort[i][0][:3] ] )
            rtnGeo.append( bloc_i )
    scl  = _BLOCK_SCALE * labelSort[0][1]
    blc = o3d.geometry.TriangleMesh.create_box( scl, scl, scl )
    for i in range(3):
        objXfrm[i,3] += hf-(scl/2.0)
    blc.transform( objXfrm )
    blc.paint_uniform_color( _CLR_TABLE[ labelSort[0][0][:3] ] )
    rtnGeo.extend( [blc,] )
    return rtnGeo

        
def generate_belief_geo( beliefs ):
    """ Draw geometry for the given `beliefs` """
    geo = table_and_origin_geo( axesScale = _BLOCK_SCALE*2.0 )
    for bel in beliefs:
        geo.extend( reading_geo( bel ) )
    # o3d.visualization.draw_geometries( geo )
    return geo
    