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
    
class Pose: 
    """ A named object, it's pose, and the surface that supports it """
    num = count()
    def __init__( self, name, pose, surf ):
        self.name = name
        self.pose = pose
        self.surf = surf # WARNING: I do not like that this is part of the predicate!
        self.index = next( self.num )
    # @property
    # def value( self ):
    #     return self.pose

class BodyConf:
    """ A robot configuration """
    def __init__( self, name, config ):
        self.name = name
        self.cnfg = config


class BodyGrasp:
    """ Enough info to pick up something """
    def __init__( self, body, grasp_pose, approach_pose ):
        self.body          = body
        self.grasp_pose    = grasp_pose
        self.approach_pose = approach_pose


class MagpieWorld:
    """ Container for Objects """
    def __init__( self ):
        self.objects = {}

    def add_object( self, pose ):
        self.objects[ pose.name ] = pose

    def get_object_pose( self, name ):
        if name in self.objects:
            self.objects[ name ].pose
        else:
            return None
        

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
_APPROACH_ABOVE_M = 0.070

def get_blocks( robot, detector ):
    """ Get block poses from YOLO """
    _, rgbdImage = detector.real.get_PCD()
    depthImage, colorImage = rgbdImage.depth, rgbdImage.color
    urPose = robot.get_tcp_pose()
    return detector.getBlocksFromImages( colorImage, depthImage, urPose, display = False )


def get_pose_stream( robot, detector, world ):
    """ Return a function that returns poses """
    # NOTE: ??? This function assumes that the camera, detector, and robot were initialized ???

    def stream_func( *args ):
        """ A function that returns poses """

        blockName, spprtName = args
        print( "Stream Args:", args )

        blocks = get_blocks( robot, detector )
        
        print( "!HEY!, The pose stream was evaluated!" )

        if len( blocks ) and ( blockName in _BLOCK_NAMES ):
            i    = _BLOCK_NAMES.index( blockName )
            pose = blocks[i].worldFrameCoords
            obj  = Pose( blockName, pose, _SUPPORT_NAME )
            world.add_object( obj )
            print( "Instantiating a", blockName )
            yield (obj,) # WARNING: HACK, The supporting object is hardcoded
        else:
            yield tuple()

    return stream_func


# def get_grasp_stream( robot )
def get_grasp_stream( world ):
    """ Return a function that returns grasps """

    examplePose = np.array( [[ 8.07158441e-04,  9.99998327e-01,  1.64138068e-03, -3.13368658e-01],
                             [ 9.99999674e-01, -8.07102857e-04, -3.45258112e-05, -5.94166567e-02],
                             [-3.32009904e-05,  1.64140802e-03, -9.99998652e-01,  7.18983894e-02],
                             [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]] )
    exOrient = examplePose[0:3,0:3]
    
    def stream_func( *args ):
        """ A function that returns grasps """
        objName = args[0]
        objPose = world.get_object_pose( objName )
        if objPose is not None:

            grasp_pose = objPose.copy()
            grasp_pose[0:3,0:3] = exOrient

            approach_pose = grasp_pose.copy()
            approach_pose[2,3] += _APPROACH_ABOVE_M
            
            grasp = BodyGrasp( objName, grasp_pose, approach_pose )
            print( "Got a grasp!\n", grasp_pose )
            return (grasp,)
            
        else:
            return tuple()
    
        # for arg in args:
        #     print( "Grasp Arg:", arg )
        #     print( dir( arg ) )
        #     print()
        # return tuple()

    return stream_func

########## PROBLEM SETUP ###########################################################################

def pddlstream_from_problem( robot, detector, world ):
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
        'sample-pose': from_gen_fn( get_pose_stream( robot, detector, world ) ), # WARNING: UNSURE OF THE WRAPPER PURPOSE
        'sample-grasp': from_gen_fn( get_grasp_stream( world ) ),
    }

    print( "About to create problem ... " )
    
    return PDDLProblem( domain_pddl, constant_map, stream_pddl, stream_map, init, goal )



########## MAIN ################################################################################

if __name__ == "__main__":

    robot, camera, detector = magpie_init()
    print( "Hardware ON!" )
    world = MagpieWorld()

    parser = create_parser()
    args = parser.parse_args()
    print('Arguments:', args)

    try:
        problem = pddlstream_from_problem( robot, detector, world )
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
    
    