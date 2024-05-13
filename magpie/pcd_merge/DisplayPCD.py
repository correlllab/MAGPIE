from typing import Tuple

import open3d as o3d
import numpy as np

import sys
sys.path.append("../../")
from magpie.motor_code import Motors
from magpie.realsense_wrapper import RealSense
from magpie.ur5 import UR5_Interface

import ObjectDetection as ob
import TaskPlanner as tp

# def get_USB_port_with_desc(descStr):
#     match = None
#     for port, desc, hwid in sorted (serial.tools.list_ports.comports()):
#         if descStr in desc:
#             match = port
#             break
#     return match

# these values come from the p variable in the getPose() function in ur5.py
pos_left = np.array(
    [-0.5269949100183583, -0.3469025118056417, -0.42664206867293003, 1.0785479404822593, -1.2311688772413145, 0.9136153632688694])

# TODO: the robot moves way too fast, from pos_left to pos_right. I need to figure out how to slow this down
pos_right = np.array(
    [-0.5118882409685005, -0.2752918558943991, -0.3151485950374825, 0.6660300666777781, -1.7252081496925555, 1.1933488314662384])

positions_array = [
    pos_left,
    pos_right,
]


def get_pcd_at_multiple_positions(robot: UR5_Interface, camera: RealSense) -> Tuple[any, list]:
    pcds = []
    rgbds = []
    for pos in positions_array:
        print("pos: ", pos)
        robot.moveL(pos)
        pcd, rgbd = camera.getPCD()
        pcds.append(pcd)
        rgbds.append(ob.TorchImage(robot.getPose(), rgbd.color, rgbd.depth))
        o3d.visualization.draw_geometries([pcd])

    o3d.visualization.draw_geometries(pcds)

    merged_point_cloud = o3d.geometry.PointCloud()  # creates an empty pcd object

    # TODO: Question 1. Is this even possible? If this is possible, we don't need code changes. I don't think this is possible
    for pc in pcds:
        merged_point_cloud += pc

    return pcds, rgbds


try:
    robotIP = "192.168.0.4"
    # con = rtde_control.RTDEControlInterface(robotIP)
    # rec = rtde_receive.RTDEReceiveInterface(robotIP)
    servoPort = "/dev/ttyACM0"
    # servoPort = get_USB_port_with_desc("OpenRB") # this and the method are from Magpie
    gripperController = Motors(servoPort)
    gripperController.torquelimit(600)  
    gripperController.speedlimit(100)
    ur = UR5_Interface(robotIP)
    ur.gripper = gripperController
    ur.start()
    ur.gripper = gripperController
    print("UR5 + Gripper Interface Established")
    real = RealSense()
    real.initConnection()
    try:
        detector = ob.ObjectDetection(real, None, moveRelative=True)
        print("Detector established")
    except Exception as e:
        detector.real.pipe.stop()
        raise (e)
    urPose = ur.getPose()
    jointAngles = ur.get_joint_angles()
    print("Joint Angles: ", jointAngles * 180 / np.pi)
    # pcd, rgbdImage = detector.real.getPCD()  # HERE
    pcd, rgbds = get_pcd_at_multiple_positions(ur, real)

    # switch get_pcd_at_multiple_positions method to return list of pcds
    # use displayPCD from RealSense class

    blocks = detector.getBlocksFromImages(rgbds)

    # need a method to get the best set of blocks from blocksList

    planner = tp.TaskPlanner(blocks)
    # goalDict = {"on":[("blueBlock", "on":[("redBlock","yellowBlock")])]}
    goalDict = {"on": [("redBlock", "yellowBlock")]}
    steps = planner.generatePlan(goalDict)
    print(steps)
    for block in blocks:
        print(f"{block.name} - {list(block.gripperFrameCoords)}")
    detector.displayWorld(pcd, blocks)
    ur.stop()
except Exception as e:
    ur.stop()
    real.disconnect()
    raise (e)
