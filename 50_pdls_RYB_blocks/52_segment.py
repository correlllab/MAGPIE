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

### Local ###
sys.path.append( "../pddlstream/" )

########## RANSAC SEGMENTATION #####################################################################

class PlanarEnv:
    """ Represent objects sitting on a table """
    def __init__( self ):
        """ Load point cloud """
        self.pcd = None
        self.obj = []

    def remove_plane( self, PCD ):
        """ Get a PC with the table removed """
        self.pcd = PCD
        plane_model, inliers = pcd.segment_plane(
            distance_threshold = 0.010,
            ransac_n = 3,
            num_iterations = 1000
        )
        self.pcd = self.pcd.select_down_sample( inliers, invert = True )

    def cluster_objects( self ):
        """ Cluster the plane-free PCD """
        with o3d.utility.VerbosityContextManager( o3d.utility.VerbosityLevel.Debug ) as cm:
            labels = np.array( pcd.cluster_dbscan( eps=0.020, min_points=10, print_progress=True ) )
        N_obj    = int( labels.max()+1 )
        clusters = [ list() for _ in range(N_obj) ]
        print( labels.shape() )
        for i, label in enumerate( labels ):
            clusters[ label ].append(i)
        for cluster in clusters:
            self.obj.append( self.pcd.select_down_sample( cluster ) )
            
        print( f"Found {len(self.obj)} objects!, {[len(o) for o in self.obj]}" )

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
        # exit()

def magpie_shutdown( robot, camera ):
    """ Start MAGPIE-related hardware """
    robot.stop()
    camera.stop()