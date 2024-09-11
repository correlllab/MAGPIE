import sys
sys.path.append("../../")
from magpie.motor_code import Motors
import magpie.ur5 as ur5
import numpy as np
import spatialmath as sm
import copy
import time
import math

sleepRate = 0.75

def get_world_frame(gripperFrameCoords, ur, z_offset=0.08):
    # given a goal position in gripper coords returns the displacements from the current pose in world coords
    xB,yB,zB = gripperFrameCoords
    # TODO: stop hardcoding a Z-stop position
    # maybe dynamically
    # subtract 0.165 from block position in gripper frame to account for gripper length
    # in METERS
    # zB -= 0.155
    zB -= z_offset
    currentPose = ur.get_tcp_pose() # 4x4 homogenous transform matrix
    # print(f"Current Pose:\n{currentPose*1000}")
    print(currentPose)
    R = currentPose[:3,:3]
    # pX,pY,pZ = tuple(currentPose.t)
    pX,pY,pZ = tuple(currentPose[:3, 3])
    # xB,yB,zB here is the block position in the gripper frame which is aligned with the optoforce frame
    P_goal = np.matmul(R,np.array([xB,yB,zB]).T)  # relative position of the block in world coordinates
    print(f"P_goal:\n{P_goal}")
    dX,dY,dZ = tuple(P_goal) # quantities and directions the the gripper frame should be incremented to be centered at the block
    return dX,dY,dZ

def moveToBlock(blockPos, ur):
    # would be better if this was block object
    # :blockPos is coord in gripper frame
    dX,dY,dZ = get_world_frame(blockPos, ur) # goalPose in world coordinates
    homePose = ur.getPose()
    dZ  += 7/1000 # up 7 mm to avoid hitting lower block
    goal1 = copy.deepcopy(homePose)
    goal1.t[2] += dZ
    ur.moveL(goal1)
    time.sleep(sleepRate)
    goal2 = goal1
    goal2.t[0] += dX
    goal2.t[1] += dY
    ur.moveL(goal2)
    time.sleep(sleepRate)

# move on the X, Y first, then Z
def move_to(pos, ur, z_offset=0.08):
    # would be better if this was block object
    # :blockPos is coord in gripper frame
    dX,dY,dZ = get_world_frame(pos, ur, z_offset=z_offset) # goalPose in world coordinates
    homePose = ur.getPose()
    goal1 = copy.deepcopy(homePose)
    goal1.t[0] += dX
    goal1.t[1] += dY
    ur.moveL(goal1)
    time.sleep(sleepRate)
    goal2 = goal1
    goal2.t[2] += dZ
    ur.moveL(goal2)
    time.sleep(sleepRate)

def move_to_L(pos, ur, z_offset=0.08):
    # would be better if this was block object
    # :blockPos is coord in gripper frame
    dX,dY,dZ = get_world_frame(pos, ur, z_offset=z_offset) # goalPose in world coordinates
    homePose = ur.getPose()
    goal1 = copy.deepcopy(homePose)
    goal1.t[0] += dX
    goal1.t[1] += dY
    # time.sleep(sleepRate)
    goal2 = goal1
    goal2.t[2] += dZ
    ur.moveL(goal2)
    # time.sleep(sleepRate)

# move up on Z first, then X, Y
def move_back(homePose, ur):
    currentPose = ur.getPose()
    dX,dY,dZ = tuple(homePose.t - currentPose.t)
    # Move in Z Axis first
    goal2 = copy.deepcopy(currentPose)
    goal2.t[2] += dZ
    ur.moveL(goal2)
    time.sleep(sleepRate)

    # Move in the XY Plane back to home
    goal3 = copy.deepcopy(goal2)
    goal3.t[0] += dX
    goal3.t[1] += dY
    ur.moveL(goal3)
    time.sleep(sleepRate)

def moveBackFromBlock(homePose, ur):
    currentPose = ur.getPose()
    # Move up 3 mm to avoid raise block to prevent friction from toppling lower block
    goal1 = copy.deepcopy(currentPose)
    goal1.t[2] -= 3/1000
    ur.moveL(goal1)
    time.sleep(sleepRate)
    currentPose = ur.getPose()
    dX,dY,dZ = tuple(homePose.t - currentPose.t)
    # Move in the XY Plane then Z Axis
    goal2 = copy.deepcopy(currentPose)
    goal2.t[0] += dX
    goal2.t[1] += dY
    ur.moveL(goal2)
    time.sleep(sleepRate)
    # Move in Z Axis back to home
    goal3 = copy.deepcopy(goal2)
    goal3.t[2] += dZ
    ur.moveL(goal3)
    time.sleep(sleepRate)

# @param pos 3d cartesian coordinate
def grab(pos):
    # orig (x, y, z): (array([ 0.13508298, -0.00355787,  0.43300185]),
    # modified (y, -x, z): [-0.00355787, -0.13508298,  0.43300185]
    # orig2: (array([ 0.04913553, -0.00977856,  0.36643119]),
    grabPos = pos

    try:
        robotIP = "192.168.0.6"
        con = rtde_control.RTDEControlInterface(robotIP)
        rec = rtde_receive.RTDEReceiveInterface(robotIP)
        servoPort = "/dev/ttyACM0"
        gripperController = Motors(servoPort)
        gripperController.torquelimit(600) # used to be 600
        gripperController.speedlimit(100)
        ur = ur5.UR5_Interface()
        ur.gripperController = gripperController
        try:
            ur.c = con
            ur.r = rec
            ur.gripperController = gripperController
        except Exception as e:
            raise(e)
        else:
            print("UR5 + Gripper Interface Established")

        # print(f"res: {projectToWorldCoords(res)} ")
        # ur.openGripper() # Open gripper
        # ur.testRoutine()
        homePose = ur.getPose()
        x_mod = 0.0
        y_mod = 0.0
        z_mod = 0.0
        moveToBlock(grabPos, ur)
        print("Done moving to block")
        ur.closeGripper(9)
        time.sleep(sleepRate)
        moveBackFromBlock(homePose, ur)
        ur.openGripper()
        gripperController.openGripper()
        gripperController.disconnect()
        ur.c.disconnect()
        ur.r.disconnect()
    except Exception as e:
        gripperController.openGripper()
        gripperController.disconnect()
        ur.c.disconnect()
        ur.r.disconnect()
        raise(e)

def grabStrat2(pos):
    # orig (x, y, z): (array([ 0.13508298, -0.00355787,  0.43300185]),
    # modified (y, -x, z): [-0.00355787, -0.13508298,  0.43300185]
    # orig2: (array([ 0.04913553, -0.00977856,  0.36643119]),
    grabPos = pos

    try:
        robotIP = "192.168.0.6"
        con = rtde_control.RTDEControlInterface(robotIP)
        rec = rtde_receive.RTDEReceiveInterface(robotIP)
        servoPort = "/dev/ttyACM0"
        gripperController = Motors(servoPort)
        gripperController.torquelimit(600) # used to be 600
        gripperController.speedlimit(100)
        ur = ur5.UR5_Interface()
        ur.gripperController = gripperController
        try:
            ur.c = con
            ur.r = rec
            ur.gripperController = gripperController
        except Exception as e:
            raise(e)
        else:
            print("UR5 + Gripper Interface Established")

        # print(f"res: {projectToWorldCoords(res)} ")
        # ur.openGripper() # Open gripper
        # ur.testRoutine()
        homePose = ur.getPose()
        x_mod = 0.0
        y_mod = 0.0
        z_mod = 0.0
        moveToBlock(grabPos, ur)
        print("Done moving to block")
        ur.closeGripper(9)
        time.sleep(sleepRate)
        moveBackFromBlock(homePose, ur)
        ur.openGripper()
        gripperController.openGripper()
        gripperController.disconnect()
        ur.c.disconnect()
        ur.r.disconnect()
    except Exception as e:
        gripperController.openGripper()
        gripperController.disconnect()
        ur.c.disconnect()
        ur.r.disconnect()
        raise(e)

    # strategy 2: close both fingers until contact, then rigidify and close

    m1 = gripperController.Motor1
    m2 = gripperController.Motor2
    # error margin of 10 pos units allowed
    m1.set_ccw_compliance_margin(10)
    # m1.set_ccw_compliance_slope(6)
    m1.set_cw_compliance_margin(10)
    # m1.set_cw_compliance_slope(6)

    m2.set_ccw_compliance_margin(10)
    m2.set_cw_compliance_margin(10)
    gripperController.torquelimit(100)

    # bring gripper to near object
    gripperController.position(50) # 45cm
    time.sleep(0.8)

    # set torque to be 5% (this is enough to move downward, not enough torque to move up)
    m1.set_torque_limit(75)
    m2.set_torque_limit(75)
    pos1 = m1.get_present_position()
    pos2 = m2.get_present_position()
    m1_stop, m2_stop = False, False

    load_thresh = 65

    for i in range(0, 200):
        if not m1_stop:
            m1.set_goal_position(pos1 + i)
        if not m2_stop:
            m2.set_goal_position(pos2 - i)
        time.sleep(0.01)
        # print(m1.get_present_position())
        load1 = m1.get_load()
        load2 = m2.get_load()
        aload1 = load1
        aload2 = load2
        if load1 >= 1023:
            aload1 = load1 - 1023
        if load2 >= 1023:
            aload2 = load2 - 1023
        # print(f"pos1: {pos1 + i } load: {aload1}")
        # print(f"pos2: {pos2 + i } load: {aload2}")
        if aload1 > load_thresh and not m1_stop: #corresponds to >0.8% load
            m1.set_goal_position(pos1 + i)
            print(f"stopping m1 at {pos1 + i} with load {aload1}")
            m1_stop = True
        if aload2 > load_thresh and not m2_stop: #corresponds to >0.8% load
            m2.set_goal_position(pos2 - i)
            print(f"stopping m2 at {pos2 - i} with load {aload2}")
            m2_stop = True

    # make gripper
    print("setting gripper to high torque")
    m1.set_ccw_compliance_margin(0)
    m1.set_cw_compliance_margin(0)
    m2.set_ccw_compliance_margin(0)
    m2.set_cw_compliance_margin(0)
    m1.set_torque_limit(500)
    m2.set_torque_limit(500)
    time.sleep(0.1)

    # close grasp
    print ("closing gripper")
    width = 30
    dTheta = gripperController.distance2theta(width)

def grabStrat1(pos):
    # strategy 1: close 1 finger until contact, then make rigid and close 2nd finger
    m1 = gripperController.Motor1
    m2 = gripperController.Motor2
    # error margin of 10 pos units allowed
    m1.set_ccw_compliance_margin(10)
    # m1.set_ccw_compliance_slope(6)
    m1.set_cw_compliance_margin(10)
    # m1.set_cw_compliance_slope(6)


    # bring m1 to near object
    # m1 reset
    m1.set_torque_limit(100)
    m1.set_goal_position(380)
    print(m1.get_present_position()) # validate margin
    time.sleep(0.8)

    # set torque to be 5% (this is enough to move downward, not enough torque to move up)
    m1.set_torque_limit(75)
    pos = m1.get_present_position()
    for i in range(0, 200):
        m1.set_goal_position(pos + i)
        # print(m1.get_present_position())
        load = m1.get_load()
        abs_load = load
        if load >= 1023:
            abs_load = load - 1023
        print(f"pos: {pos + i } load: {abs_load}")
        if abs_load > 55: #corresponds to >0.8% load
            break

    time.sleep(0.1)

    # set current pos to new goal (to avoid extraneous movement)
    m1.set_goal_position(m1.get_present_position())
    time.sleep(0.4)


    # make m1 rigid
    # m1 config
    m1.set_ccw_compliance_margin(0)
    # m1.set_ccw_compliance_slope(6)
    m1.set_cw_compliance_margin(0)
    # m1.set_cw_compliance_slope(6)
    m1.set_torque_limit(200)
    time.sleep(0.3)

    print(m2.get_present_position())

    # close grasp
    m2.set_goal_position(500)

