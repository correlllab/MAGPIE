########## INIT ####################################################################################

##### Imports #####

### Standard ###
import os
from ast import literal_eval
from datetime import datetime
from time import sleep
from pprint import pprint

### Special ###
import numpy as np

### Local ###
from TaskPlanner import VisualCortex, match_name
from magpie import ur5
from magpie.poses import repair_pose, translation_diff, vec_unit
from graphics.homog_utils import posn_from_xform
from task_planning.symbols import p_symbol_inside_workspace_bounds
from task_planning.belief import extract_class_dist_in_order, get_D405_FOV_frustum, p_sphere_inside_plane_list

### Config ###
from env_config import _BLOCK_SCALE, _BLOCK_NAMES, _NULL_NAME


##### Constants #####

_ACCEPT_RAD     = 1.5*_BLOCK_SCALE
_HS             = _BLOCK_SCALE/2.0
_BLOCK_RAD_M    = np.sqrt( 3.0 * _HS**2 )
_CONF_BLC_POSES = [
    np.array([[ 1.,     0.,     0.,    -0.286,],
              [ 0.,     1.,     0.,    -0.291,],
              [ 0.,     0.,     1.,     _HS  ,],
              [ 0.,     0.,     0.,     1.   ,],]),

    np.array([[ 1.,     0.,     0.,    -0.297,],
              [ 0.,     1.,     0.,    -0.413,],
              [ 0.,     0.,     1.,     _HS  ,],
              [ 0.,     0.,     0.,     1.   ,],]),

    np.array([[ 1.,     0.,     0.,    -0.304,],
              [ 0.,     1.,     0.,    -0.537,],
              [ 0.,     0.,     1.,     _HS  ,],
              [ 0.,     0.,     0.,     1.   ,],]),
]



########## DATA COLLECTION CONTROLLER ##############################################################


class Confuser:
    """ Collect data to compute the confusion matrix """

    ##### Init ############################################################

    def open_file( self ):
        """ Set the name of the current file """
        dateStr     = datetime.now().strftime("%m-%d-%Y_%H-%M-%S")
        self.outNam = f"Confusion-Data_{dateStr}.txt"
        self.outFil = open( os.path.join( self.outDir, self.outNam ), 'w' )


    def __init__( self ):
        """ Set data collection params & robot connection """
        self.world  = VisualCortex( noViz = False ) 
        self.robot  = ur5.UR5_Interface() 
        self.outDir = "data/ConfMatx/"
        self.pausMv = 1.0
        self.pausSh = 3.0
        self.datLin = list()
        self.robot.start()
        self.open_file()


    def shutdown( self ):
        """ Stop the Perception Process and the UR5 connection """
        self.world.stop()
        self.robot.stop()


    ##### Data Collection #################################################

    def take_shot( self ):
        """ Get all `obs`ervations at a `cam`era pose """
        return {
            'obs' : self.world.get_perc_output(),
            'cam' : self.robot.get_cam_pose().tolist(),
        }
    
    def tcp_from_cam_pose( self, camPose ):
        return camPose.dot( np.linalg.inv( np.array( self.robot.camXform ) ) )
    
    def take_shot_at( self, desCamPose, capture = False, userCheck = True ):
        """ Place the camera at a given location and run perception pipeling """
        tcpPose = self.tcp_from_cam_pose( desCamPose )
        print( f"\n{tcpPose}\n" )
        if userCheck:
            ans = breakpoint( "Verify SANE pose!" )
        else:
            ans = 'y'
        if ('n' not in str( ans ).lower()):
            self.robot.moveL( tcpPose, asynch = False )
            sleep( self.pausMv )
            rtnDat = self.take_shot()
            sleep( self.pausSh )
            if capture:
                self.datLin.append( rtnDat )
            return rtnDat
        else:
            print( f"User REJECTED move to \n{tcpPose}" )
            return {'obs':None, 'cam':None,}


    def dump_to_file( self, openNext = False ):
        """ Write all data lines to a file """
        self.outFil.writelines( [f"{str(line)}\n" for line in self.datLin] )
        self.outFil.close()
        if openNext:
            self.datLin = list()
            self.open_file()


    def run_shot_program( self, program, actual = None, openNext = False, userCheck = True ):
        """ Go to every camera pose in the `program` and take a shot, Dump data when complete """
        print( "\n##### Run Program #####\n" )
        outN = self.outNam
        if actual is not None:
            self.datLin.append( [[pair[0],pair[1].tolist()] for pair in actual] )
        for i, pose in enumerate( program ):
            print( f"\n## Pose {i+1} ##" )
            self.take_shot_at( pose, capture = True, userCheck = userCheck )
        self.dump_to_file( openNext )
        print( f"\n##### Program COMPLETE --wrote-> {self.outDir}{outN} #####\n" )

    

########## DATA PROCESSING #########################################################################

def read_program_output( inPath, actual = False ):
    """ Read all data lines written by `dump_to_file` """
    rtnData = []
    with open( inPath, 'r' ) as f:
        lines = f.readlines()
        if actual:
            pairs  = literal_eval( lines[0] )
            rtnAct = list()
            for name, pose in pairs:
                rtnAct.append([
                    name, 
                    np.array( pose )
                ])
            alines = lines[1:]
        else:
            alines = lines
        for line in alines:
            if len( line ) > 5:
                shot   = dict()
                struct = literal_eval( line )
                shot['cam'] = np.array( struct['cam'] ).reshape( (4,4,) )
                shot['obs'] = list()
                for obs_i in struct['obs'].values():
                    pose_i = shot['cam'].dot( np.array( obs_i['Pose'] ).reshape( (4,4,) ) )
                    pose_i[0:3,0:3] = np.eye(3)
                    shot['obs'].append({
                        'Probability' : obs_i[ 'Probability' ],
                        'Count'       : obs_i[ 'Count'       ],
                        'Time'        : obs_i[ 'Time'        ],
                        'Pose'        : pose_i                ,
                    })
                rtnData.append( shot )
    if actual:
        return rtnData, rtnAct
    else:
        return rtnData
        

def get_truth_inside_of_camera_frustum( camPose, actual ):
    rtnActual = list()
    bounds    = get_D405_FOV_frustum( camPose )
    for (gndNam, gndPose) in actual:
        if # FIXME

    

########## DATA GATHERING MOTION PLANS #############################################################

def camera_pose_from_target_offset( trgtPose, offsetVec ):
    """ Get a "fingers flat" camera pose with the wrist kinda pointed towards the chair """
    zBasis  = -vec_unit( offsetVec )
    yBasis  =  vec_unit( np.cross( zBasis, [0.0, 0.0, 1.0,] ) )
    xBasis  = np.cross( yBasis, zBasis )
    positn  = posn_from_xform( trgtPose ) + np.array( offsetVec )
    rtnPose = np.eye(4)
    rtnPose[0:3,0] = xBasis
    rtnPose[0:3,1] = yBasis
    rtnPose[0:3,2] = zBasis
    rtnPose[0:3,3] = positn
    return repair_pose( rtnPose )


def make_program_poses( bgnTrgtPose, offsetVec, moveDir, dStep_m, Nstep ):
    """ Build a pose program using `camera_pose_from_target_offset` """
    moveDir = vec_unit( moveDir )
    stepVec = moveDir * dStep_m
    bgnPose = camera_pose_from_target_offset( bgnTrgtPose, offsetVec )
    rtnProg = [bgnPose,]
    for _ in range( 1, Nstep ):
        pose_i = rtnProg[-1].copy()
        pose_i[0:3,3] += stepVec
        rtnProg.append( pose_i )
    return rtnProg


if __name__ == "__main__":

    _POSES_PER_HEIGHT = 11
    _HEIGHT_SEQ_M     = [0.200, 0.300, 0.400]


    dataPaths = [
        "data/ConfMatx/Confusion-Data_07-29-2024_20-40-18.txt",
        "data/ConfMatx/Confusion-Data_07-29-2024_20-44-25.txt",
        "data/ConfMatx/Confusion-Data_07-29-2024_20-48-34.txt",
        "data/ConfMatx/Confusion-Data_07-29-2024_21-01-16.txt",
        "data/ConfMatx/Confusion-Data_07-29-2024_21-05-37.txt",
        "data/ConfMatx/Confusion-Data_07-29-2024_21-09-54.txt",
    ]

    N_obs   = 0
    N_xcl   = 0
    totConf = dict()
    for h in _HEIGHT_SEQ_M:
        totConf[ h ] = dict()
        for bName in _BLOCK_NAMES:
            totConf[ h ][ bName ] = list()
    


    for dPath in dataPaths:
        print( f"\n########## Reading {dPath} ##########\n" )
        shots, actual = read_program_output( dPath, actual = True )
        # 1. For each height level
        for i, camHeight in enumerate( _HEIGHT_SEQ_M ):
            confHght = dict()
            for bName in _BLOCK_NAMES:
                confHght[ bName ] = list()
            # 2. For each shot at this height level, Fetch the shot
            for j in range( _POSES_PER_HEIGHT ):
                k    = (i*_POSES_PER_HEIGHT)+j
                shot = shots[k]
                # 3. For each observation in this shot, fetch and analyze
                for obs in shot['obs']:
                    N_obs += 1
                    pose_k = obs['Pose']
                    dist_k = extract_class_dist_in_order( obs['Probability'], insertZero = True )
                    # 4. Only consider observations with reasonable poses
                    if p_symbol_inside_workspace_bounds( pose_k ):
                        # 5. Attempt to match with the ground truth
                        found = False
                        for (gndNam, gndPose) in actual:
                            if translation_diff( gndPose, pose_k ) <= _ACCEPT_RAD:
                                gndName = match_name( gndNam )
                                confHght[ gndName ].append( dist_k )
                                found = True
                                break
                        # 6. If it is not a match, Then it was a hallucination
                        if not found:
                            confHght[ _NULL_NAME ].append( dist_k )
                    else:
                        N_xcl += 1
            for bName in _BLOCK_NAMES:
                totConf[ camHeight ][ bName ].extend( confHght[ bName ] )

        
    N_con = 0
    for k1 in totConf.keys():
        for k2 in totConf[ k1 ].keys():
            N_con += len( totConf[ k1 ][ k2 ] )

    print( f"Processed {N_obs} observations, Excluded {N_xcl}, Recorded {N_con} confusions!, Check OK: {(N_obs-N_xcl) == N_con}" )

    os.system( 'kill %d' % os.getpid() ) 




    # bgnTrgt = _CONF_BLC_POSES[0].copy()
    # bgnTrgt[1,3] += 0.075

    # prog = list()

    # for dist in [0.200, 0.300, 0.400]:
    #     prog.extend(
    #         make_program_poses( bgnTrgt, vec_unit( [-0.200,0.200,dist] )*(dist+_BLOCK_RAD_M), [0,-1,0], 0.050, 11 )
    #     )

    # # actBlc = list( zip(['ylw','blu','NOT'], _CONF_BLC_POSES) ) # Config 1
    # # actBlc = list( zip(['blu','grn','NOT'], _CONF_BLC_POSES) ) # Config 2
    # # actBlc = list( zip(['ylw','grn','NOT'], _CONF_BLC_POSES) ) # Config 3
    # # actBlc = list( zip(['blu','ylw','NOT'], _CONF_BLC_POSES) ) # Config 4
    # # actBlc = list( zip(['grn','blu','NOT'], _CONF_BLC_POSES) ) # Config 5
    # actBlc = list( zip(['grn','ylw','NOT'], _CONF_BLC_POSES) ) # Config 6

    # # camTst = camera_pose_from_target_offset( _CONF_BLC_POSES[0], vec_unit( [-1,1,1] )*0.200 )
    # # print( camTst )

    # ctrl = Confuser()
    # ctrl.run_shot_program( prog, actual = actBlc, openNext = False, userCheck = False )
    # ctrl.shutdown()

    # # tcpCom = repair_pose( camAct.dot( np.transpose( np.array( rbt.camXform ) ) ) )
    # # tcpCom = camAct.dot( np.linalg.inv( np.array( rbt.camXform ) ) ) 
    # # print( tcpAct )
    # # print( tcpCom )
    
    # # print( f"Computed Difference: {np.linalg.norm( tcpAct - tcpCom )}" )
    # # rbt.stop()


    # # prog = [ _CONF_CAM_POSE_ANGLED1.copy(), ]
    # # for _ in range( int(40/5) ):
    # #     pose_i = prog[-1].copy()
    # #     pose_i[1,3] -= 0.050
    # #     prog.append( pose_i )
    
    # # ctrl = Confuser()
    # # ctrl.run_shot_program( prog, openNext = False, userCheck = True )
    # # ctrl.shutdown()

    # poseData = read_program_output( "data/ConfMatx/Confusion-Data_07-29-2024_17-14-35.txt" )
    # cnfPoses = []

    # 

    # for shot in poseData:
    #     for obs in shot['obs']:
    #         pose_j = obs['Pose']
    #         found  = False
    #         for pose_k in cnfPoses:
    #             if translation_diff( pose_j, pose_k ) <= _ACCEPT_RAD:
    #                 found = True
    #                 pose_k[0:3,3] = (posn_from_xform( pose_j ) + posn_from_xform( pose_k ))/2.0
    #                 break
    #         if (not found) and p_symbol_inside_workspace_bounds( pose_j ):
    #             cnfPoses.append( pose_j )
    # for pose_k in cnfPoses:
    #     print( pose_k )
    #     print()