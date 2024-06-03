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
    [-0.5728491986844327, -0.2665255758472088, -0.2932157256924612, -0.009479043067194798, -2.6199047172066074, 1.0365085648368406])

# TODO: the robot moves way too fast, from pos_left to pos_right. I need to figure out how to slow this down
pos_right = np.array(
    [-0.4909784424437883, -0.3229772924246015, -0.449609224618071, 0.8283579388044375, 2.034872485189882, -2.149809095513047])

positions_array = [
    pos_left,
    pos_right,
]


def get_pcd_at_multiple_positions(robot: UR5_Interface, camera: RealSense) -> Tuple[any, list]:
    pcds = []
    rgbds = []
    for pos in positions_array:
        print("pos: ", pos)
        robot.moveL_with_coords(pos)
        pcd, rgbd = camera.getPCD()
        pcds.append(pcd)
        rgbds.append(ob.TorchImage(robot.getPose(), rgbd.color, rgbd.depth))

    camera.displayPCD(pcds) # TODO: test if this solves point cloud problem

    merged_point_cloud = o3d.geometry.PointCloud()  # creates an empty pcd object

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
    # ur.gripper = gripperController
    ur.start()
    ur.gripper = gripperController
    print("UR5 + Gripper Interface Established")
    ur.open_gripper()
    print("gripper open")
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
    position_coords = ur.get_pose_coords()
    print("position coords: ", position_coords)
    # pcd, rgbdImage = detector.real.getPCD()  # HERE
    pcd, rgbds = get_pcd_at_multiple_positions(ur, real)
    
    # switch get_pcd_at_multiple_positions method to return list of pcds
    # use displayPCD from RealSense class

    blocks = detector.getBlocksFromImages(rgbds, ur)

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
