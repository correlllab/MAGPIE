
from __future__ import print_function

from pddlstream.algorithms.meta import solve, create_parser
from pddlstream.language.generator import from_gen_fn, from_fn, empty_gen, from_test, universe_test
from pddlstream.utils import read, INF, get_file_path, find_unique, Profiler, str_from_object, negate_test
from pddlstream.language.constants import print_solution, PDDLProblem
from examples.pybullet.tamp.streams import get_cfree_approach_pose_test, get_cfree_pose_pose_test, get_cfree_traj_pose_test, \
    move_cost_fn, get_cfree_obj_approach_pose_test

########## DOMAIN OBJECTS ##########################################################################
_BLOCK_NAMES  = ['redBlock', 'ylwBlock', 'bluBlock',]
_SUPPORT_NAME = 'table'

class Pose:
    """ A named object, it's pose, and the surface that supports it """
    def __init__( self, name, pose, surf ):
        self.name = name
        self.pose = pose
        self.surf = surf # WARNING: I do not like that this is part of the predicate!


class BodyConf:
    """ A robot configuration """
    def __init__( self, name, config ):
        self.name = name
        self.cnfg = config



########## MAGPIE ##################################################################################

import sys
sys.path.append( "../" )
from magpie.ur5 import UR5_Interface
from Perception import DepthCam, ObjectDetection

def magpie_init():
    """ Start MAGPIE-related hardware """
    # QUESTION: SHOULD THERE BE A SINGLETON MAGPIE "MANAGER" OBJECT?
    robot = UR5_Interface()
    robot.start()
    camera = DepthCam()
    detector = ObjectDetection( camera, None, moveRelative = True) # FIXME: I HATE THIS ENTANGLEMENT
    return robot, camera, detector

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

    def stream_func():
        """ A function that returns poses """
        
        blocks = get_blocks( robot, detector )
        print( "!HEY!, The pose stream was evaluated!" )

        while True: # WARNING: I DO NOT KNOW WHY THIS PREVENTS REDUNDANT STATES
            for i, blockName in enumerate( blockNames ):
                pose = blocks[i].worldFrameCoords
                yield (Pose( blockName, pose, _SUPPORT_NAME ),) # WARNING: HACK, The supporting object is hardcoded
            blocks = get_blocks( robot, detector )

    return stream_func


########## PROBLEM SETUP ###########################################################################



def pddlstream_from_problem( robot, detector, movable=[], teleport=False, grasp_name='top' ):
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
        'sample-pose': get_pose_stream( robot, detector ),
    }

    return PDDLProblem( domain_pddl, constant_map, stream_pddl, stream_map, init, goal )
            
