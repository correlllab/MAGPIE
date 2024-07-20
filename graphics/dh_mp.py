"""
ur_mp.py
Minimum viable motion planning & visualization for Universal Robotics

2022-02-16, NOTE: At this time all poses are assumed to be expressed in the robot's base frame
"""

########## INIT ####################################################################################

##### Imports #####
from random import random
import numpy as np
from numpy import sin, cos, isnan, arctan2, pi, sqrt, arccos, arcsin
from homog_utils import ( p_point_above_plane, R_z, homog_xform, R_krot, R_x, bases_from_xform, arrayify, SIGN, posn_from_xform, 
                          maxdex, vec_angle_between, vec_weighted_avg, apply_homog_to_posn_vec )

##### Constants #####
_ZERO_THRESH       = 0.00000001
_ANGLE_MARGIN      = pi/(180*4)
_DEFAULT_VEC_COLOR = [255/255, 106/255, 0/255]

##### Env. Settings #####
np.set_printoptions( precision=3 )



########## DH PARAMETERS (FORWARD KINEMATICS) ######################################################


def dh_link_homog( theta, alpha, a, d ):
    """ Return a homogeneous transformation that corresponds to the given Denavit-Hartenberg parameters """
    
    # Link Displacements
    aVec = np.array( [ [ a ], [0.0], [0.0] ] )
    dVec = np.array( [ [0.0], [0.0], [ d ] ] )
    
    # Rotation
    rotn = R_z( theta ).dot( R_x( alpha ) )
    
    # Displacement
    aDsp = rotn.dot( aVec )
    # tDsp = dVec + aDsp
    tDsp = np.add( dVec, aDsp )
    
    # Orientation
    linkT = np.eye( 4 )
    linkT[ 0:3 , 0:3 ] = rotn
    linkT[ 0:3 , 3   ] = tDsp.flatten()
    
    # Return the homogenous coordinates of the end of this link
    return linkT


# def FK_DH( dhParamsMatx, qConfig, baseLink = None, baseQ = None ):
def FK_DH( dhParamsMatx, qConfig ):
    """ Calculate the end effector pose in homogeneous coords using DH parameters and joint config """
    DH        = arrayify( dhParamsMatx )
    qC        = arrayify( qConfig.copy() )    
    DH        = np.insert( DH, 0, [0.0 for _ in range(3)], axis = 0 )
    qC        = np.insert( qC, 0, 0.0 )
    N         = len( DH ) # Number of links defined by DH
    effectorT = np.eye(4) # ------------ Output pose
    
    for i, row in enumerate( DH ):
        (alpha , a , d) = row
        theta           = qC[i]
        effectorT       = effectorT.dot( dh_link_homog( theta , alpha , a , d ) )
    
    return effectorT


# def FK_DH_chain( dhParamsMatx, qConfig, baseLink = None, baseQ = None ):
def FK_DH_chain( dhParamsMatx, qConfig ):
    """ Calculate all link poses in homogeneous coords using DH parameters and joint config """
    DH = dhParamsMatx.copy()
    qC = qConfig.copy()
    
#     if baseLink is None:
#         DH = np.insert( DH, 0, [0.0 for _ in range(3)], axis = 0 )
#     else:
#         DH = np.insert( DH, 0, baseLink, axis = 0 )
        
#     if baseQ is None:
#         qC = np.insert( qC, 0, 0.0 )
#     else:
#         qC = np.insert( qC, 0, baseQ )

    DH        = np.insert( DH, 0, [0.0 for _ in range(3)], axis = 0 )
    qC        = np.insert( qC, 0, 0.0 )
    N         = len( DH ) # Number of links defined by DH
    effectorT = dh_link_homog( qC[0] , *DH[0] ) # ------------ Output pose
    chain     = []
    
    for i, row in enumerate( DH ):
        (alpha , a , d) = row
        theta           = qC[i]
        effectorT       = effectorT.dot( dh_link_homog( theta , alpha , a , d ) )
        chain.append( effectorT.copy() )
    
    return chain



########## INVERSE KINEMATICS ######################################################################

##### UR5 Solver #####

def analytical_IK_UR5( dhParamsMatx, targetHomog, q6_des = 0.0 ):
    """ Closed-form Inverse Kinematics for UR5 """
    # https://github.com/atenpas/universal_robot/blob/kinetic-devel/ur_kinematics/src/ur_kin.cpp
    N    = len( dhParamsMatx )
    T00  = targetHomog[0][0]
    T01  = targetHomog[0][1]
    T02  = targetHomog[0][2]
    T03  = targetHomog[0][3]
    T10  = targetHomog[1][0]
    T11  = targetHomog[1][1]
    T12  = targetHomog[1][2]
    T13  = targetHomog[1][3]
    T20  = targetHomog[2][0]
    T21  = targetHomog[2][1]
    T22  = targetHomog[2][2]
    T23  = targetHomog[2][3]
    a2   = dhParamsMatx[2-1][1]
    a3   = dhParamsMatx[3-1][1]
    d1   = dhParamsMatx[1-1][2]
    d4   = dhParamsMatx[4-1][2]
    d5   = dhParamsMatx[5-1][2]
    d6   = dhParamsMatx[6-1][2]
    A    = d6*T12 - T13
    B    = d6*T02 - T03
    R    = A*A + B*B
    nmsl = 0
    qSln = [ [float('nan') for _ in range(N)] for _ in range(8) ]
    
    #################### shoulder rotate joint (q1) ####################
    q1 = [float('nan') for _ in range(2)]
    
    if abs(A) < _ZERO_THRESH:
        
        if abs( abs(d4) - abs(B)) < _ZERO_THRESH:
            div = -SIGN(d4)*SIGN(B)
        else:
            div = -d4/B
        
        _arcsin = arcsin( div )
        
        if abs( _arcsin ) < _ZERO_THRESH:
            _arcsin = 0.0
        
        if _arcsin < 0.0:
            q1[0] = _arcsin + 2.0*pi
        else:
            q1[0] = _arcsin
        
        q1[1] = pi - _arcsin
        
    elif abs(B) < _ZERO_THRESH:
        
        if abs( abs(d4) - abs(A) ) < _ZERO_THRESH:
            div = SIGN(d4)*SIGN(A)
        else:
            div = d4/A
        
        _arccos = arccos( div )
        q1[0]   = _arccos
        q1[1]   = 2.0*pi - _arccos
        
    elif d4*d4 > R:
        print( "WARN, analytical_IK_UR5: NO Solution!" )
        return [ float('nan') for _ in range(N) ]
    
    else:
        _arccos = arccos( d4 / sqrt(R) ) 
        _arctan = arctan2( -B, A )
        pos     =  _arccos + _arctan
        neg     = -_arccos + _arctan
        
        if abs(pos) < _ZERO_THRESH:
            pos = 0.0
        if abs(neg) < _ZERO_THRESH:
            neg = 0.0
        
        if pos >= 0.0:
            q1[0] = pos
        else:
            q1[0] = 2.0*pi + pos
        
        if neg >= 0.0:
             q1[1] = neg
        else:
             q1[1] = 2.0*pi + neg
                
    #################### wrist 2 joint (q5) ####################
    q5 = [ [float('nan') for _ in range(2)] for _ in range(2) ]
    
    for i in range(2):
        
        numer = T03*sin(q1[i]) - T13*cos(q1[i]) - d4
        
        if abs( abs(numer) -  abs(d6)) < _ZERO_THRESH:
            div = SIGN(numer) * SIGN(d6)
        else:
            div = numer / d6
        
        _arccos  = arccos(div)
        q5[i][0] = _arccos
        q5[i][1] = 2.0*pi - _arccos
    
    #################### q2, q3, q4, q6 ####################
    for i in range(2):
        for j in range(2):
            
            c1 = cos( q1[i]    )
            s1 = sin( q1[i]    )
            c5 = cos( q5[i][j] )
            s5 = sin( q5[i][j] )
            
            ########## wrist 3 joint (q6) ##########
            if abs(s5) < _ZERO_THRESH:
                q6 = q6_des
            else:
                q6 = arctan2( SIGN(s5)*(-1)*(T01*s1 - T11*c1) , 
                              SIGN(s5)*(+1)*(T00*s1 - T10*c1) )
                if abs( q6 ) < _ZERO_THRESH:
                    q6 = 0.0
                if q6 < 0.0:
                    q6 += 2.0*pi
                    
            ########## RRR joints (q2,q3,q4) ##########
            q2   = [float('nan') for _ in range(2)]
            q3   = [float('nan') for _ in range(2)]
            q4   = [float('nan') for _ in range(2)]
            c6   = cos( q6 )
            s6   = sin( q6 )
            x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1))
            x04y = c5*(T20*c6 - T21*s6) - T22*s5
            p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1
            p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6)
            c3   = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3)
            if abs( abs(c3) - 1.0) < _ZERO_THRESH:
                c3 = SIGN( c3 )
            elif abs(c3) > 1.0:
                # TODO: NO SOLUTION
                # print('continue')
                continue
            _arccos = arccos(c3)
            q3[0]   = _arccos
            q3[1]   = 2.0*pi - _arccos
            denom   = a2*a2 + a3*a3 + 2*a2*a3*c3
            s3      = sin( _arccos )
            A       = (a2 + a3*c3)
            B       = a3*s3
            q2[0]   = arctan2( (A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom )
            q2[1]   = arctan2( (A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom )
            c23_0   = cos( q2[0] + q3[0] )
            s23_0   = sin( q2[0] + q3[0] )
            c23_1   = cos( q2[1] + q3[1] )
            s23_1   = sin( q2[1] + q3[1] )
            q4[0]   = arctan2( c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0 )
            q4[1]   = arctan2( c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1 )
            
            ########## Compute & Store All Sol'ns ##########
            for k in range(2):
                
                if abs(q2[k]) < _ZERO_THRESH:
                    q2[k] = 0.0
                elif q2[k] < 0.0:
                    q2[k] += 2.0*pi
                
                if abs(q4[k]) < _ZERO_THRESH:
                    q4[k] = 0.0
                elif q4[k] < 0.0:
                    q4[k] += 2.0*pi
                    
                qSln[nmsl][0] = q1[i]
                qSln[nmsl][1] = q2[k]
                qSln[nmsl][2] = q3[k]
                qSln[nmsl][3] = q4[k]
                qSln[nmsl][4] = q5[i][j]
                qSln[nmsl][5] = q6
                nmsl += 1
                
    return [sln for sln in qSln if not isnan( sln[0] )]



##### Pose Error #####


def position_error( homog1, homog2 ):
    """ Return the euclidean distance between coordinates `homog1` and `homog2` """
    return np.linalg.norm( np.subtract(
        posn_from_xform( homog1 ) , 
        posn_from_xform( homog2 )
    ) ) 


def orientation_error( homog1, homog2 ):
    """ Return the greatest angular error between any of the principle axes in coordinates `homog1` and `homog2` """
    # NOTE: This is meant to be a cheap version of Siciliano quaternion error
    return max(
        vec_angle_between( homog1[0:3,0], homog2[0:3,0] ) ,
        vec_angle_between( homog1[0:3,1], homog2[0:3,1] ) ,
        vec_angle_between( homog1[0:3,2], homog2[0:3,2] )
    )


def pose_error( homog1, homog2 ):
    """ Return the position and orientation errors as a tuple """
    return ( position_error( homog1, homog2 ), orientation_error( homog1, homog2 ), )


########## MANIPULATION HELPERS ####################################################################


def vel_jacobian( dhParamsMatx, qConfig, decRound = 5 ):
    """ Get the velocity Jacobian for a `dhParamsMatx` robot in the `qConfig` configuration """
    # NOTE: The base frame is assumed static and therefore does not contribute to the calculation
    
    # 0. Alloc J_v matrix
    N  = len( dhParamsMatx )
    Jv = np.zeros( (6,N) )
    
    # 1. Calc frames for all links
    chain = FK_DH_chain( dhParamsMatx, qConfig )
    d_n   = posn_from_xform( chain[-1] )
    # 2. For each joint, there will be a column to the matrix
    for i, link in enumerate( dhParamsMatx ):
        # 3. Fetch link measurements
        d_i           = posn_from_xform( chain[i] )
        d_i2n         = np.subtract( d_n, d_i )
        # [xB, yB, zB]  = bases_from_xform( chain[i+1] )
        [xB, yB, zB]  = bases_from_xform( chain[i] )
        # 4. Calculate J_v column
        Jv[0:3,i] = np.cross( zB, d_i2n ) # Linear vel contribution
        Jv[3:6,i] = zB # ------------------ Rotational vel contribution
        
    return Jv.round( decimals = decRound )


def manip_score( dhParamsMatx, qConfig, decRound = 5 ):
    """ Yoshikawa's manipulability index """
    Jv = vel_jacobian( dhParamsMatx, qConfig, decRound )
    return sqrt(  np.linalg.det( Jv.dot( np.transpose( Jv ) ) )  )



########## DH MODELS ###############################################################################

UR5distMod = 0.10915 # Offset shows Link 2 and it's COM where it belongs in space

UR5_DH = np.array([
     # alpha  ,  a      , d
    [  pi/2.0,  0.0     ,  0.089159    ], 
    [  0.0   , -0.425   ,  UR5distMod  ], 
    [  0.0   , -0.39225 , -UR5distMod  ], 
    [  pi/2.0,  0.0     ,  0.10915     ], 
    [ -pi/2.0,  0.0     ,  0.09465     ], 
    [  0.0   ,  0.0     ,  0.0823      ], 
])

UR5e_DH = np.array([
     # alpha  ,  a    , d
    [  pi/2.0,  0.0  , 0.1625 ], 
    [  0.0   , -0.425, 0.0    ], 
    [  0.0   , -0.392, 0.0    ], 
    [  pi/2.0,  0.0  , 0.1333 ], 
    [ -pi/2.0,  0.0  , 0.0997 ], 
    [  0.0   ,  0.0  , 0.0996 ], 
])



########## MASS & BALANCE ##########################################################################

##### Mass Models #####

UR5_COM = np.array([
    # x     ,  y      ,  z
    [ 0.0000, -0.02561,  0.00193             ],
    [ 0.2125,  0.0000 ,  0.11336-UR5distMod  ],
    [ 0.1500,  0.0000 ,  0.0265              ],
    [ 0.0000, -0.0018 ,  0.01634             ],
    [ 0.0000,  0.0018 ,  0.01634             ],
    [ 0.0000,  0.0000 , -0.001159            ],
])

UR5_mass = np.array([
    # kg
    3.7000 ,
    8.3930 ,
    2.3300 ,
    1.2190 ,
    1.2190 ,
    0.1879 ,
])


##### Robot Center of Mass #####

def robot_COM_for_q( dhParamsMatx, qConfig, comLst, massLst, rtnTotalWeight = 1 ):
    """ Return the COM in the robot base frame """
    position = np.zeros( (len(dhParamsMatx),3) )
    frames   = FK_DH_chain( dhParamsMatx, qConfig )
    for i, frm in enumerate( frames[1:] ):
        position[i] = apply_homog_to_posn_vec( frm, comLst[i] )
    return vec_weighted_avg( position, massLst, rtnTotalWeight = rtnTotalWeight )