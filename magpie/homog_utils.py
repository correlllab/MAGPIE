########## INIT ####################################################################################

##### Imports #####
from random import random
import numpy as np
from numpy import sin, cos

##### Env. Settings #####
np.set_printoptions( precision=3 )


########## CONTAINERS ##############################################################################


def maxdex( sequence ):
    """ Return the index of the maximum element of the sequence """
    return sequence.index( max( sequence ) )


def arrayify( iterArg ):
    """ Attempt to convert to a numpy array """
    if isinstance( iterArg, np.ndarray ):
        return iterArg.copy()
    if isinstance( iterArg, list ):
        return np.array( iterArg )
    try:
        len( iterArg )
        if not isinstance( iterArg, str ):
            return np.array( list( iterArg ) )
        else:
            raise ValueError( f"arrayify: String {iterArg} cannot be convered into a Numpy array!" )
    except:
        raise ValueError( f"arrayify: Argument {iterArg} cannot be convered into a Numpy array!" )


########## RANDOM SAMPLING #########################################################################


def sample_unfrm_real( rMin, rMax ):
    """ Sample from a uniform distribution [ rMin , rMax ) """
    span = abs( rMax - rMin )
    return random() * span + rMin


def sample_unfrm_N( rMin, rMax, N ):
    """ Sample from a uniform distribution [ rMin , rMax ) """
    rtnArr = []
    span = abs( rMax - rMin )
    return np.array( [ (random()*span+rMin) for _ in range(N) ] )


def sample_bbox_uniform( bbox ):
    """ Sample uniformly from a hypercuboid `bbox` """
    return np.array( [ sample_unfrm_real( dim[0] , dim[1] ) for dim in bbox ] )


def sample_bbox_N( bbox, N ):
    """ Draw `N` samples uniformly from a hypercuboid `bbox` """
    return np.array( [ sample_bbox_uniform( bbox ) for _ in range(N) ] )


########## 3D GEOMETRY #############################################################################


##### Basic Math #####


def ver( theta ):
    """ Versine, radians """
    return 1 - cos( theta )


def vec_mag( vec ):
    """ Return the magnitude of `vec` """
    return np.linalg.norm( vec )


def vec_unit( vec ):
    """ Return the unit vector in the direction of `vec` """
    mag = np.linalg.norm( vec )
    if mag == 0.0:
        return np.zeros( vec.shape )
    else:
        return np.divide( vec, mag )
    
    
def vec_angle_between( v1, v2 ): 
    """ Returns the angle in radians between vectors 'v1' and 'v2' """
	# URL, angle between two vectors: http://stackoverflow.com/a/13849249/893511
    v1_m = vec_mag( v1 )
    v2_m = vec_mag( v2 )
    if v1_m == 0.0 or v2_m == 0.0:
        return float('nan')
    v1_u = np.divide( v1, v1_m )
    v2_u = np.divide( v2, v2_m )
    # print( np.dot( v1_u, v2_u) )
    dotVc = np.dot( v1_u, v2_u )
    if abs( dotVc ) - 1.0 <= 1e-7:
        dotVc = dotVc / abs( dotVc )
    angle = np.arccos( dotVc )
    if np.isnan( angle ):
        if ( v1_u == v2_u ).all():
            return 0.0
        else:
            return np.pi
    return angle


def SIGN( x ):
    """ Return +/-1 corresponding to the sign of `x` """
    return int(x > 0) - int(x < 0)


def vec_weighted_avg( vectors, weights, rtnTotalWeight = 1 ):
    """ Compute the weighted average vector where the sum of contributions is 1.0 """
    vectors = arrayify( vectors )
    wTotal  = sum( weights )
    wNormd  = np.divide( weights, wTotal )
    rtnVec  = np.zeros( vectors[0].size )
    for i, v_i in enumerate( vectors ):
        rtnVec = np.add(  rtnVec,  np.multiply( v_i, wNormd[i] )  )
    if rtnTotalWeight:
        return rtnVec, wTotal
    else:
        return rtnVec


##### Planes #####


def p_point_above_plane( pntXYZ, planeXYZD, inclusive = 0 ):
    """ Return True if `pntXYZ` lies above `planeXYZD` in Z+ """
    pnt = np.ones( (4,) )
    pnt[:3] = pntXYZ
    if inclusive:
        return (pnt.dot( planeXYZD ) >= 0.0)
    else:
        return (pnt.dot( planeXYZD ) >  0.0)



##### Rotations, R^3 #####

def R_x( theta ):
    """ Return the 3x3 rotation matrix that corresponds to a rotation by 'theta' about the principal X-axis """
    return np.array( [ [1 ,  0            ,  0           ] ,
                       [0 ,  cos( theta ) , -sin( theta )] , 
                       [0 ,  sin( theta ) ,  cos( theta )] ] )


def R_y( theta ):
    """ Return the 3x3 rotation matrix that corresponds to a rotation by 'theta' about the principal Y-axis """
    return np.array( [ [cos( theta ) , 0 , -sin( theta )] ,
                       [0            , 1 ,  0           ] , 
                       [sin( theta ) , 0 ,  cos( theta )] ] )


def R_z( theta ):
    """ Return the 3x3 rotation matrix that corresponds to a rotation by 'theta' about the principal Z-axis """
    return np.array( [ [cos( theta ) , -sin( theta ) , 0] , 
                       [sin( theta ) ,  cos( theta ) , 0] ,
                       [0            ,  0            , 1] ] )


def R_krot( k, theta ):
    """ Return the 3x3 rotation matrix for the given angle 'theta' and axis 'k' """
    k = vec_unit( k )
    # ver = marchhare.marchhare.ver
    return np.array( [ 
        [ k[0]*k[0]*ver(theta) + cos(theta)      , k[0]*k[1]*ver(theta) - k[2]*sin(theta) , k[0]*k[2]*ver(theta) + k[1]*sin(theta) ] , 
        [ k[1]*k[0]*ver(theta) + k[2]*sin(theta) , k[1]*k[1]*ver(theta) + cos(theta)      , k[1]*k[2]*ver(theta) - k[0]*sin(theta) ] , 
        [ k[2]*k[0]*ver(theta) - k[1]*sin(theta) , k[2]*k[1]*ver(theta) + k[0]*sin(theta) , k[2]*k[2]*ver(theta) + cos(theta)      ] 
    ] )


def shortest_rot_btn_vecs( v1, v2 ):
    """ Return the Quaternion representing the shortest rotation from vector 'v1' to vector 'v2' """
    return R_krot( np.cross( v1, v2 ), vec_angle_between( v1, v2 ) )



##### Homogeneous Coordinates, R^16 #####


def posn_from_xform( xform ):
    """ Get the position vector from the homogeneous transformation """
    return np.array( xform[0:3,3] )


def R_from_xform( xform ):
    """ Get the rotation matrix from the homogeneous transformation """
    return np.array( xform[0:3,0:3] )


def bases_from_xform( xform ):
    """ Return the basis vector for the transformation """
    xBasis = np.array( xform[0:3,0] )
    yBasis = np.array( xform[0:3,1] )
    zBasis = np.array( xform[0:3,2] )
    return xBasis, yBasis, zBasis


def homog_xform( rotnMatx = None, posnVctr = None ): 
    """ Return the combination of rotation matrix and displacement vector as a 4x4 homogeneous transformation matrix """
    xform = np.eye(4)
    if rotnMatx is not None:
        xform[0:3,0:3] = rotnMatx
    if posnVctr is not None:
        xform[0:3,3] = posnVctr
    return xform


def invert_xform( xform ):
    """ Return the transform T^{-1} that is the reverse of T. """
    return np.linalg.inv( xform )


def xform_from_vertical_at_position( positionVec, directionVec ):
    """ Return transform that will correctly place an "upwards-facing" graphics model """
    return homog_xform( 
        rotnMatx = shortest_rot_btn_vecs( [0.0,0.0,1.0], directionVec ),
        posnVctr = positionVec
    )


def change_to_child_frame( childHomog, labTarget ):
    """ Express `labTarget` in the lab frame in the reference frame `childHomog` directly under lab frame """
    return homog_xform( 
        rotnMatx = None, 
        posnVctr = np.subtract( posn_from_xform( labTarget ) , posn_from_xform( childHomog ) )
    )


def apply_homog_to_posn_vec( xform, posn ):
    """ Transform the position with `xform` """
    position = homog_xform( posnVctr = posn )
    xfmdPosn = xform.dot( position )
    return posn_from_xform( xfmdPosn )



########## 2D GEOMETRY #############################################################################


def eq( op1, op2 ):
    return (abs( op1-op2 ) < 1e-7)


def elemw( i, iterable ): 
    """ Return the 'i'th index of 'iterable', wrapping to index 0 at all integer multiples of 'len(iterable)' , Wraps forward and backwards """
    seqLen = len( iterable )
    if i >= 0:
        return iterable[ i%seqLen ]
    else:
        revDex = abs(i)%seqLen
        if revDex == 0:
            return iterable[0]
        return iterable[ seqLen-revDex ]


def winding_num_2D( point , polygon ):
    """ Find the winding number of a point with respect to a polygon , works for both CW and CCWpoints """
    # NOTE: Function assumes that the points of 'polygon' are ordered. Algorithm does not work if they are not
    # This algorithm is translation invariant, and can handle convex, nonconvex, and polygons with crossing sides.
    # This algorithm does NOT handle the case when the point lies ON a polygon side. For this problem it is assumed that
    #there are enough trial points to ignore this case
    # This works by shifting the point to the origin (preserving the relative position of the polygon points), and
    #tracking how many times the positive x-axis is crossed
    w = 0.0
    v_i = []
    x = lambda index: elemw( index , v_i )[0] # We need wrapping indices here because the polygon is a cycle
    y = lambda index: elemw( index , v_i )[1]
    for vertex in polygon: # Shift the point to the origin, preserving its relative position to polygon points
        v_i.append( np.subtract( vertex , point ) )
    for i in range(len(v_i)): # for each of the transformed polygon points, consider segment v_i[i]-->v_i[i+1]
        if y(i) * y(i + 1) < 0: # if the segment crosses the x-axis
            r = x(i) + ( y(i) * ( x(i + 1) - x(i) ) ) / ( y(i) - y(i + 1) ) # location of x-axis crossing
            if r > 0: # positive x-crossing
                if y(i) < 0: 
                    w += 1 # CCW encirclement
                else:
                    w -= 1 #  CW encirclement
        # If one of the polygon points lies on the x-axis, we must look at the segments before and after to determine encirclement
        elif ( eq( y(i) , 0 ) ) and ( x(i) > 0 ): # v_i[i] is on the positive x-axis, leaving possible crossing
            if y(i) > 0:
                w += 0.5 # possible CCW encirclement or switchback from a failed CW crossing
            else:
                w -= 0.5 # possible CW encirclement or switchback from a failed CCW crossing
        elif ( eq( y(i + 1) , 0 ) ) and ( x(i + 1) > 0 ): # v_i[i+1] is on the positive x-axis, approaching possible crossing
            if y(i) < 0:
                w += 0.5 # possible CCW encirclement pending
            else:
                w -= 0.5 # possible  CW encirclement pending
    return w

def point_in_poly_2D( point , polygon ):
    """ Return True if the 'polygon' contains the 'point', otherwise return False, based on the winding number """
    return not ( winding_num_2D( point , polygon ) == 0.0 )  # The winding number gives the number of times a polygon encircles a point 