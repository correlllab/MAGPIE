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

def get_world_frame(gripperFrameCoords, ur, z_offset=0.11463):
    '''
    z_offset: z_offset from camera to wrist joint on ur5
    '''
    # given a goal position in gripper coords returns the displacements from the current pose in world coords
    xB,yB,zB = gripperFrameCoords
    zB -= z_offset
    currentPose = ur.get_tcp_pose() # 4x4 homogenous transform matrix
    R = currentPose[:3,:3]
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


