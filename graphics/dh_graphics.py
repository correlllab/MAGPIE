"""
ur_graphics.py
Minimum viable motion planning & visualization for Universal Robotics

2022-02-16, NOTE: At this time all poses are assumed to be expressed in the robot's base frame
"""

########## INIT ####################################################################################

##### Imports #####
import numpy as np
import open3d as o3d
from homog_utils import bases_from_xform, posn_from_xform, arrayify, R_krot, homog_xform
from dh_mp import FK_DH_chain, dh_link_homog, robot_COM_for_q

##### Constants #####
_DEFAULT_VEC_COLOR = [255/255, 106/255, 0/255]

########## PLOTTING ################################################################################


##### Plotting Helpers #####


def xform_geo_list( geoLst, xform ):
    """ Transform all items in the list """
    for geo in geoLst:
        geo.transform( xform )
        
        
def color_geo_list( geoLst, color ):
    """ Transform all items in the list """
    for geo in geoLst:
        geo.paint_uniform_color( color )
        
        
##### Serial Manipulators #####


def plot_DH_robot( dhParamsMatx, qConfig, axesScale = 0.050, getDrawList = 0, suppressTable = 0 ):
    """ Plot the kinematic chain represented by `dhParamsMatx` in `qConfig`, using Open3d """
    # 0. Set up drawing accounting
    geo     = []
    index   = 0
    lastPnt = posn_from_xform( np.eye(4) )
    addSeg  = [ lastPnt.copy().flatten(), ]
    addIdx  = []
    # 1. Generate link frames
    chain = FK_DH_chain( dhParamsMatx, qConfig ) #, baseLink = baseLink, baseQ = baseQ )
    # 2. Fetch base link bases
    [alpha, a, d] = [0.0 for _ in range(3)]
    theta         = 0.0
    [xB, yB, zB]  = bases_from_xform( dh_link_homog( theta, alpha, a, d ) )    
    
    # 3. For each link: Create geometries for frame, a-segment, and d-segment
    for i, frm in enumerate( chain ):
        
        # 4. Create frame geo
        f_i = o3d.geometry.TriangleMesh.create_coordinate_frame( size = axesScale )
        f_i.transform( frm )
        geo.append( f_i )
        
        if i > 0: 
            # 5. Fetch link measurements
            [alpha, a, d] = dhParamsMatx[i-1]
            theta         = qConfig[i-1]
            
            # 6. Paint 'd', if present
            if abs(d) > 0.0:
                nextPnt = np.add( lastPnt, np.multiply(zB, d) )
                addSeg.append( nextPnt.copy().flatten() )
                addIdx.append( [index, index+1] )
                index += 1
                lastPnt = nextPnt.copy()
        
            # 7. Paint 'a', if present
            if abs(a) > 0.0:
                nextPnt = np.add(
                    lastPnt,
                    np.multiply(
                        R_krot( xB, alpha ).dot( R_krot( zB, theta ) ).dot( xB ), 
                        a
                    )
                )
                addSeg.append( nextPnt.copy().flatten() )
                addIdx.append( [index, index+1] )
                index += 1
                lastPnt = nextPnt.copy()
            
            # 8. Fetch frame bases
            [xB, yB, zB]  = bases_from_xform( frm )
    
    # 9. Create link geo
    geo.append( o3d.geometry.LineSet( 
        o3d.utility.Vector3dVector( np.array(addSeg) ), # Vertices
        o3d.utility.Vector2iVector( np.array(addIdx) ) #- Indices
    ) )
    
    if not suppressTable:
        # 10. Create floor geo
        floor = o3d.geometry.TriangleMesh.create_box( 2.0, 2.0, 0.010 )
        floor.transform( np.array(
            [ [1.0, 0.0, 0.0, -1.0   ] ,
              [0.0, 1.0, 0.0, -1.0   ] ,
              [0.0, 0.0, 1.0, -0.010 ] ,
              [0.0, 0.0, 0.0,  1.0   ] ] 
        ) )
        geo.append( floor )

    # N. Draw
    if getDrawList:
        return geo
    else:
        o3d.visualization.draw_geometries( geo )
                
            
            
##### Robot Center of Mass #####

def plot_robot_link_COMs( dhParamsMatx, qConfig, comLst, massLst, axesScale = 0.050, suppressTable = 0 ):
    """ Draw the robot with the COM """
    geo      = plot_DH_robot( dhParamsMatx, qConfig, axesScale = axesScale, getDrawList = 1, suppressTable = suppressTable )
    position = np.zeros( (len(dhParamsMatx),3) )
    frames   = FK_DH_chain( dhParamsMatx, qConfig )
    for i, frm in enumerate( frames[1:] ):
        position[i] = apply_homog_to_posn_vec( frm, comLst[i] )
        mSphr       = o3d.geometry.TriangleMesh.create_sphere(radius=massLst[i]*axesScale*0.1, resolution=20, create_uv_map=False)
        mSphr.paint_uniform_color( [0,0,0] )
        mSphr.transform( homog_xform( posnVctr = position[i] ) )
        geo.append( mSphr )
        print(i, massLst[i])
    o3d.visualization.draw_geometries( geo )


def plot_robot_w_COM( dhParamsMatx, qConfig, comLst, massLst, axesScale = 0.050, getDrawList = 0, suppressTable = 0 ):
    """ Draw the robot with the COM """
    geo = plot_DH_robot( dhParamsMatx, qConfig, axesScale = axesScale, getDrawList = 1, suppressTable = suppressTable )
    posn, mass = robot_COM_for_q( dhParamsMatx, qConfig, comLst, massLst )
    mSphr = o3d.geometry.TriangleMesh.create_sphere(radius=mass*axesScale*0.1, resolution=20, create_uv_map=False)
    mSphr.paint_uniform_color( [0,0,0] )
    mSphr.transform( homog_xform( posnVctr = posn ) )
    geo.append( mSphr )
    if getDrawList:
        return geo
    else:
        o3d.visualization.draw_geometries( geo )
        
##### Forces and Torques #####
        
        
def get_vec_arrow_geo( origin, vec, 
                       graphicScale = 1.0, arrowFrac = 0.25, cylinderRadFrac = 0.01, coneRadFrac = 0.02,
                       color = _DEFAULT_VEC_COLOR ):
    """ Get geometry object showing `vec` as it starts from `origin` (Lab Frame) """
    mag = vec_mag( vec )
    if mag > 0.0:
        headLen = mag * graphicScale * arrowFrac
        tailLen = mag * graphicScale * (1.0-arrowFrac)
        xform   = xform_from_vertical_at_position( origin, vec )
        arrow = o3d.geometry.TriangleMesh.create_arrow(  
            cylinder_radius = mag * cylinderRadFrac,
            cone_radius     = mag * coneRadFrac,
            cylinder_height = tailLen, 
            cone_height     = headLen, 
            resolution      = 4,  # 20
            cylinder_split  = 4, 
            cone_split      = 1
        )
        arrow.transform( xform )
        arrow.paint_uniform_color( color )
        return [arrow,]
    else:
        return []


def get_vec_spin_geo( origin, vec, graphicScale = 1.0,
                      color = _DEFAULT_VEC_COLOR ):
    """ Get a representation of a right-hand rotational quantity """
    circRad = vec_mag( vec ) * graphicScale
    if circRad > 0.0:
        xform   = xform_from_vertical_at_position( origin, vec )
        torus = o3d.geometry.TriangleMesh.create_torus( 
            torus_radius = circRad, 
            tube_radius  = circRad*0.06, 
            radial_resolution=15, tubular_resolution=15
        )
        axisV = o3d.geometry.TriangleMesh.create_cone( radius=circRad*0.20, height=circRad,      resolution=6 )
        arow1 = o3d.geometry.TriangleMesh.create_cone( radius=circRad*0.25, height=circRad*0.35, resolution=6 )
        arow2 = o3d.geometry.TriangleMesh.create_cone( radius=circRad*0.25, height=circRad*0.35, resolution=6 )

        xfrm = homog_xform( 
            posnVctr = [ 0.0, 0.0, -circRad/2.0 ]
        )
        axisV.transform( xfrm )

        xfrm = homog_xform( 
            rotnMatx = R_x( -pi/2.0 ),
            posnVctr = [ +circRad, 0.0, 0.0 ]
        )
        arow1.transform( xfrm )

        xfrm = homog_xform( 
            rotnMatx = R_x( +pi/2.0 ),
            posnVctr = [ -circRad, 0.0, 0.0 ]
        )
        arow2.transform( xfrm )
        geoLst = [ torus, axisV, arow1, arow2 ]
        xform_geo_list( geoLst, xform )
        color_geo_list( geoLst, color )
        return geoLst
    else:
        return []
