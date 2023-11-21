from __future__ import print_function

### Standard ###
import sys, os, traceback
from itertools import count
try:
    from collections import Sequence
except ImportError:
    from collections.abc import Sequence

### Special ###
import numpy as np

### Local ###
sys.path.append( "../pddlstream/" )
from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.generator import from_gen_fn, from_fn, empty_gen, from_test, universe_test
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
from pddlstream.language.constants import print_solution, PDDLProblem

########## DOMAIN OBJECTS ##########################################################################
_BLOCK_NAMES  = ['redBlock', 'ylwBlock', 'bluBlock',]
_SUPPORT_NAME = 'table'

# class Pose(Sequence): # ??? WHY DOES THIS HAVE TO BE A SEQUENCE ???
#     """ A named object, it's pose, and the surface that supports it """
#     num = count()
#     def __init__( self, name, pose, surf ):
#         self.name = name
#         self.pose = pose
#         self.surf = surf # WARNING: I do not like that this is part of the predicate!
#         self.index = next(self.num)
#     def __getitem__( self, i ):
#         return self.pose
#     def __len__( self ):
#         return 1
#     @property
#     def value(self):
#         return self.pose
    
class Pose: # ??? WHY DOES THIS HAVE TO BE A SEQUENCE ???
    """ A named object, it's pose, and the surface that supports it """
    num = count()
    def __init__( self, name, pose, surf ):
        self.name = name
        self.pose = pose
        self.surf = surf # WARNING: I do not like that this is part of the predicate!
        self.index = next( self.num )
    @property
    def value(self):
        return self.pose



class BodyConf:
    """ A robot configuration """
    def __init__( self, name, config ):
        self.name = name
        self.cnfg = config



########## MAGPIE ##################################################################################

import sys, time
sys.path.append( "../" )
from magpie.ur5 import UR5_Interface
from Perception import DepthCam, ObjectDetection

def magpie_init():
    """ Start MAGPIE-related hardware """
    # QUESTION: SHOULD THERE BE A SINGLETON MAGPIE "MANAGER" OBJECT?
    for _ in range(3):
        try:
            robot = UR5_Interface()
            robot.start()
            camera = DepthCam()
            detector = ObjectDetection( camera, None, moveRelative = True) # FIXME: I HATE THIS ENTANGLEMENT
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



########## STREAMS #################################################################################

def get_blocks( robot, detector ):
    """ Get block poses from YOLO """
    _, rgbdImage = detector.real.get_PCD()
    depthImage, colorImage = rgbdImage.depth, rgbdImage.color
    urPose = robot.get_tcp_pose()
    return detector.getBlocksFromImages( colorImage, depthImage, urPose, display = False )


def get_pose_stream( robot, detector ):
    """ Return a function that returns poses """
    # NOTE: ??? This function assumes that the camera, detector, and robot were initialized ???

    blockNames  = ['redBlock', 'ylwBlock', 'bluBlock',]

    def stream_func( *args ):
        """ A function that returns poses """

        print( "Stream Args:", args )

        blocks = get_blocks( robot, detector )
        
        print( "!HEY!, The pose stream was evaluated!" )

        # while True:
        if len( blocks ):
            for i, blockName in enumerate( blockNames ):
                pose = blocks[i].worldFrameCoords
                print( "Instantiating a", blockName )
                yield (Pose( blockName, pose, _SUPPORT_NAME ),) # WARNING: HACK, The supporting object is hardcoded
        else:
            yield tuple()
            
        # blocks = get_blocks( robot, detector )

    return stream_func


########## PROBLEM SETUP ###########################################################################

def pddlstream_from_problem( robot, detector ):
    """ Set up a PDDLStream problem with the UR5 """

    domain_pddl = read(get_file_path(__file__, 'domain.pddl'))
    stream_pddl = read(get_file_path(__file__, 'stream.pddl'))
    constant_map = {}
    print( "Read files!" )

    print( 'Robot:', robot.get_name() )
    conf = BodyConf( robot.get_name(), robot.get_joint_angles() )
    init = [('CanMove',),
            ('Conf', conf),
            ('AtConf', conf),
            ('HandEmpty',)]
    print( "Robot grounded!" )

    for body in _BLOCK_NAMES:
        init += [('Graspable', body),]
        init += [('Stackable', body, _SUPPORT_NAME)]

    goal = ('and',
            ('AtConf' , conf), # Move back to the original config
            ('Holding', 'redBlock'), # Be holding the red block
    )

    stream_map = {
        'sample-pose': from_gen_fn( get_pose_stream( robot, detector ) ),
    }

    print( "About to create problem ... " )
    
    return PDDLProblem( domain_pddl, constant_map, stream_pddl, stream_map, init, goal )



########## MAIN ################################################################################

if __name__ == "__main__":

    robot, camera, detector = magpie_init()
    print( "Hardware ON!" )

    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    try:
        problem = pddlstream_from_problem( robot, detector )
        print( "Problem created!" )
    
        solution = solve( problem, algorithm="incremental", unit_costs=True, success_cost=INF )
        print( "Solver has completed!" )
        print_solution( solution )
        plan, cost, evaluations = solution

    except KeyboardInterrupt as e:
        print( f"Solver was stopped by the user at {time.time()}!" )
    
    except Exception as e:
        print( "Solving failed due to:", e )
        print(traceback.format_exc())
        magpie_shutdown( robot, camera )
        exit()

    
        

    magpie_shutdown( robot, camera )
    print( "Hardware OFF!" )
    
    