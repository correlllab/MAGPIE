########## INIT ####################################################################################

import sys

import numpy as np
import vispy
from vispy import scene
from vispy.visuals import transforms
from vispy.color import Color
import numpy as np

### Local ###
# from graphics.dh_mp import UR5_DH
# from graphics.dh_graphics import plot_DH_robot
from graphics.homog_utils import posn_from_xform
from graphics.homog_utils import vec_unit
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


def look_at_matrix( target, eye, up = None ):
    """ Construct the camera transformation """
    if up is None:
        up = [0,0,1,]
    else:
        up = vec_unit( up )
    zBasis = vec_unit( np.subtract( target, eye ) )
    xBasis = vec_unit( np.cross( up, zBasis ) )
    yBasis = np.cross( zBasis, xBasis )
    rtnMtx = np.eye(4)
    rtnMtx[0:3,0] = xBasis
    rtnMtx[0:3,1] = yBasis
    rtnMtx[0:3,2] = zBasis
    rtnMtx[0:3,3] = eye
    return np.transpose( rtnMtx )

########## DRAWING FUNCTIONS #######################################################################

def table_geo():
    """ Draw the usable workspace """
    # table  = o3d.geometry.TriangleMesh.create_box( _X_WRK_SPAN, _Y_WRK_SPAN, _TABLE_THIC )
    table  = scene.visuals.Box( _X_WRK_SPAN, _TABLE_THIC, _Y_WRK_SPAN,  
                                color = [237/255.0, 139/255.0, 47/255.0, 1.0], edge_color="black" , )
    table.transform = transforms.STTransform( translate = (
        _MIN_X_OFFSET + _X_WRK_SPAN/2.0, 
        _MIN_Y_OFFSET + _Y_WRK_SPAN/2.0, 
        -_TABLE_THIC/2.0
    ) )
    return table


def vispy_geo_list_window( geoLst ):
    canvas = scene.SceneCanvas( keys='interactive', size=(1000, 900), show=True )
    # vispy.gloo.wrappers.set_state( cull_face = True )
    # vispy.gloo.wrappers.set_cull_face( mode = 'back' )

    # Set up a viewbox to display the cube with interactive arcball
    view = canvas.central_widget.add_view()
    view.bgcolor = '#ffffff'
    view.camera = 'arcball'
    view.padding = 100
    # view.camera.transform.matrix = look_at_matrix( target, eye )
    view.add( scene.visuals.XYZAxis() )

    for geo in geoLst:
        view.add( geo )
        # canvas.draw_visual( geo )
    
    # view.update()

    canvas.app.run()

# vispy_geo_list_window( [table_geo(),] )

# def UR5_geo( qConfig ):
#     """ Compute drawable geo of the UR5 in `qConfig` """
#     return plot_DH_robot( UR5_DH, qConfig, getDrawList=1, suppressTable=1 )


def reading_dict_geo( objReading, lineColor = None, baseAlpha = 1.0 ):
    """ Get geo for a single observation """
    if lineColor is None:
        lineColor = "black"
    hasLabel  = ('label' in objReading)
    labelSort = zip_dict_sorted_by_decreasing_value( objReading['labels'] )
    objXfrm   = extract_pose_as_homog( objReading['pose'], noRot = True )
    objPosn   = posn_from_xform( objXfrm ) #- hf
    if hasLabel:
        blcColor = _CLR_TABLE[ objReading['label'][:3] ]
    else:
        blcColor = _CLR_TABLE[ labelSort[0][0][:3]     ]

    blcColor = (np.array( blcColor )*baseAlpha + np.array( [1,1,1,] )*(1.0-baseAlpha)).tolist()

    block = scene.visuals.Box( _BLOCK_SCALE, _BLOCK_SCALE, _BLOCK_SCALE, 
                               color = blcColor, edge_color = lineColor )
    block.transform = transforms.STTransform( translate = objPosn )

    clnDct = dict()
    for k, v in objReading['labels'].items():
        clnDct[ k[:3] ] = np.round( v, 2 )

    text = scene.visuals.Text(
        str( clnDct ), 
        # parent = block,
        color  = 'black',
    )
    text.font_size = 3
    # text.pos = [_BLOCK_SCALE*1.5, _BLOCK_SCALE*1.5, 0.0,]
    text.pos = np.add( objPosn, [0.0, 0.0, _BLOCK_SCALE*0.65,] )

    return block, text
    
    
#     rtnGeo  = list()
    
#     wir = wireframe_box_geo( _BLOCK_SCALE, _BLOCK_SCALE, _BLOCK_SCALE )
#     wir.transform( objXfrm )
#     rtnGeo.extend( [wir,] )
    
#     for i in range(3):
#         objXfrm[i,3] -= hf
#     for i in range( 1, min( len(labelSort), len(topCrnrs) ) ):
#         prob_i = labelSort[i][1]
#         if (prob_i > 0.0):
#             scal_i  = _BLOCK_SCALE * prob_i
#             xfrm_i = topCrnrs[i-1]
#             for j in range(3):
#                 xfrm_i[j,3] -= scal_i/2.0
#             xfrm_i = objXfrm.dot( xfrm_i )
#             bloc_i = o3d.geometry.TriangleMesh.create_box( scal_i, scal_i, scal_i )
#             bloc_i.transform( xfrm_i )
#             bloc_i.paint_uniform_color(  )
#             rtnGeo.append( bloc_i )
#     scl  = _BLOCK_SCALE * labelSort[0][1]
#     blc = o3d.geometry.TriangleMesh.create_box( scl, scl, scl )
#     for i in range(3):
#         objXfrm[i,3] += hf-(scl/2.0)
#     blc.transform( objXfrm )
#     blc.paint_uniform_color( _CLR_TABLE[ labelSort[0][0][:3] ] )
#     rtnGeo.extend( [blc,] )
#     blc = o3d.geometry.
#     return rtnGeo

        
# def generate_belief_geo( beliefs ):
#     """ Draw geometry for the given `beliefs` """
#     geo = table_and_origin_geo( axesScale = _BLOCK_SCALE*2.0 )
#     for bel in beliefs:
#         geo.extend( reading_geo( bel ) )
#     # o3d.visualization.draw_geometries( geo )
#     return geo
    