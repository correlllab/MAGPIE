from __future__ import print_function

########## INIT #################################################################################

### Standard ###
import sys, os, traceback
from itertools import count
try:
    from collections.abc import Sequence
except ImportError:
    print( "IMPORT FAILED" )

### Special ###
import numpy as np
import open3d as o3d

### Local ###
sys.path.append( "../pddlstream/" )

########## RANSAC SEGMENTATION #####################################################################
from homog_utils import R_krot, homog_xform

def get_pcd_centroid( pcd ):
    """ Get the average point of the PCD """
    return np.asarray( pcd.points ).mean( axis = 0 )

def get_pcd_avg_color( pcd ):
    """ Get the average point of the PCD """
    return np.asarray( pcd.colors ).mean( axis = 0 )

def get_oriented_box_R( obb ):
    """ Get the rotation matrix for the oriented bounding box """
    R = np.eye(3)
    P = np.asarray( obb.get_box_points() )
    pi = np.around( P[0,:], 6 )
    pj = np.around( P[1,:], 6 )
    pk = np.around( P[2,:], 6 )
    X = pj - pi
    Y = pk - pi
    Z = np.cross( X, Y )
    X = X / np.linalg.norm(X)
    Y = Y / np.linalg.norm(Y)
    Z = Z / np.linalg.norm(Z)
    R[:,0] = X
    R[:,1] = Y
    R[:,2] = Z
    return R
    
    
    
    

class SimpleBlock:
    """ Use this to ground the RYB blocks """
    def __init__( self, name, pcd, pose ):
        self.name = name
        self.pcd  = pcd
        self.pose = pose
        

class PlanarEnv:
    """ Represent objects sitting on a table """
    def __init__( self ):
        """ Load point cloud """
        self.pcd = None
        self.obj = []
        self.pos = []

    def remove_plane( self, PCD ):
        """ Get a PC with the table removed """
        self.pcd = PCD
        print( "Original Cloud:   ", len(self.pcd.points) )
        self.pcd = self.pcd.voxel_down_sample( voxel_size = 0.0025)
        print( "Downsampled Cloud:", len(self.pcd.points) )
        plane_model, inliers = self.pcd.segment_plane(
            distance_threshold = 0.005,
            ransac_n = 3,
            num_iterations = 1000
        )
        print( "Plane EQ:", plane_model )
        self.pcd = self.pcd.select_by_index( inliers, invert = True )
        print( "Plane removed!", len(self.pcd.points) )

    def cluster_objects( self ):
        """ Cluster the plane-free PCD """
        print( "About to cluster ...", len(self.pcd.points) )
        with o3d.utility.VerbosityContextManager( o3d.utility.VerbosityLevel.Debug ) as cm:
            # labels = np.array( pcd.cluster_dbscan( eps=0.020, min_points=10, print_progress=True ) )
            labels = np.array( self.pcd.cluster_dbscan( 
                eps = 0.010, 
                min_points = 30, 
                print_progress=True 
            ) )
        N_obj    = int( labels.max()+1 )
        clusters = [ list() for _ in range(N_obj) ]
        print( "`labels` shape:", labels.shape )
        for i, label in enumerate( labels ):
            clusters[ label ].append(i)
        for cluster in clusters:
            cloud_i = self.pcd.select_by_index( cluster )
            if len( cloud_i.points ) > 150:
                self.obj.append( cloud_i )
            
        print( f"Found {len(self.obj)} objects!, {[len(o.points) for o in self.obj]}" )

        return self.obj

    def filter_objects( self, boxMax = 0.050 ):
        """ Filter objects by size """
        res = []
        for cloud in self.obj:
            obb = cloud.get_oriented_bounding_box()
            coordMax = max( obb.extent )
            if coordMax <= boxMax:
                res.append( cloud )
                self.pos.append( homog_xform( 
                    rotnMatx = get_oriented_box_R( obb ), 
                    posnVctr = get_pcd_centroid( cloud )  
                ) )
                # print( self.pos[-1] )
        self.obj = res
        print( f"There are {len(self.obj)} filtered objects!" )
        return res

    def get_blocks_RYB( self ):
        """ Find the RYB blocks """
        redMax = -10.0
        ylwMax = -10.0
        bluMax = -10.0
        blocks = [None,None,None]
        for i, o in enumerate( self.obj ):
            oClr = get_pcd_avg_color( o )
            print( "Average Color:", oClr )
            redVal = np.dot( oClr, [1,0,0] ) - np.linalg.norm([1,0,0]-oClr)
            ylwVal = np.dot( oClr, [1,0.8,0] ) - np.linalg.norm([1,0.8,0]-oClr)
            bluVal = np.dot( oClr, [0,0,1] ) - np.linalg.norm([0,0,1]-oClr)
            print( redVal, ylwVal, bluVal )
            if redVal > redMax:
                redMax = redVal
                blocks[0] = SimpleBlock( 'redBlock', o, self.pos[i] )
            if ylwVal > ylwMax:
                ylwMax = ylwVal
                blocks[1] = SimpleBlock( 'ylwBlock', o, self.pos[i] )
            if bluVal > bluMax:
                bluMax = bluVal
                blocks[2] = SimpleBlock( 'bluBlock', o, self.pos[i] )
        return blocks

def display_PCD_list( pcds ):
    """ Displays a list of point clouds given an array of PCDs """
    # flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
    # pcd.transform(flip_transform)
    # worldFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075)
    geo = [ o3d.geometry.TriangleMesh.create_coordinate_frame( size = 0.075 ) ]
    geo.extend( pcds )
    # print( geo )
    o3d.visualization.draw_geometries( geo )



########## MAGPIE ###############################################################################

import sys, time
sys.path.append( "../" )
from magpie.ur5 import UR5_Interface
from Perception import DepthCam #, ObjectDetection

def magpie_init():
    """ Start MAGPIE-related hardware """
    # QUESTION: SHOULD THERE BE A SINGLETON MAGPIE "MANAGER" OBJECT?
    for _ in range(3):
        try:
            robot = UR5_Interface()
            robot.start()
            camera = DepthCam()
            detector = PlanarEnv()
            print( "Init SUCCESS!" )
            return robot, camera, detector
        except RuntimeError as e:
            print( "Hardware start failed due to", e )
            time.sleep(1)
    return None, None, None

def magpie_shutdown( robot, camera ):
    """ Start MAGPIE-related hardware """
    robot.stop()
    camera.stop()



########## MAIN #################################################################################
_BLOCK_EDGE_M = 0.020
if __name__ == "__main__":

    robot, camera, detector = magpie_init()
    print( "Hardware ON!" )

    pcd, _ = camera.get_PCD()
    detector.remove_plane( pcd )
    objClouds = detector.cluster_objects()
    # display_PCD_list( objClouds )
    objClouds = detector.filter_objects( 0.040 )
    blocks    = detector.get_blocks_RYB()

    print( [block.name for block in blocks] )
    for block in blocks:
        print( block.pose )

    geo = [ o3d.geometry.TriangleMesh.create_coordinate_frame( size = 0.050 ), ]

    offset = np.zeros( (4,4) )
    offset[0:3,3] = [-_BLOCK_EDGE_M/2.0, -_BLOCK_EDGE_M/2.0, -_BLOCK_EDGE_M/2.0]
    
    geo.append( blocks[0].pcd )
    box = o3d.geometry.TriangleMesh.create_coordinate_frame( size = 0.050 )
    box.transform( blocks[0].pose )
    box.paint_uniform_color( [1,0,0] )
    geo.append( box )

    geo.append( blocks[1].pcd )
    box = o3d.geometry.TriangleMesh.create_coordinate_frame( size = 0.050 )
    box.transform( blocks[1].pose )
    box.paint_uniform_color( [1,1,0] )
    geo.append( box )

    geo.append( blocks[2].pcd )
    box = o3d.geometry.TriangleMesh.create_coordinate_frame( size = 0.050 )
    box.transform( blocks[2].pose )
    box.paint_uniform_color( [0,0,1] )
    geo.append( box )

    o3d.visualization.draw_geometries( geo )

    magpie_shutdown( robot, camera )
    print( "Hardware OFF!" )