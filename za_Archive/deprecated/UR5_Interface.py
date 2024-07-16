import numpy as np
import spatialmath as sm
import poses as poses
import time
class UR5_Interface():
    def __init__(self):
        self.robotIP = "192.168.0.6"
        # RTDEControlInterface and RTDEReceiveInterface Objects
        self.c, self.r = None, None
        # Gripper Controller Interface
        self.gripperController = None

    def getJointAngles(self):
        # Returns a 6 element numpy array of joint angles (radians)
        thetas = np.array(self.r.getActualQ()) #getActualQ returns actual joint positions
        return thetas

    def getPose(self):
        # Returns the current pose of the last frame as a SE3 Object (4 x 4 Homegenous Transform)
        p = self.r.getActualTCPPose()
        poseMatrix = self.poseVectorToMatrix(p)
        T_N = sm.SE3(poseMatrix)   # convert a pose vector to a matrix SE3 object, SE3 --> special euclidean in 3-dimensional space
        # T_N.plot(name="C")
        return T_N    # T_N is a homogenous transform

    def poseVectorToMatrix(self, poseVector):
        # Converts poseVector into an SE3 Object (4 x 4 Homegenous Transform)
        # poseVector is a 6 element list of [x, y, z, rX, rY, rZ]
        T_N = sm.SE3(poses.pose_vec_to_mtrx(poseVector))
        return T_N

    def poseMatrixToVector(self, poseMatrix):
        # Converts poseMatrix into a 6 element list of [x, y, z, rX, rY, rZ]
        # poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
        return poses.pose_mtrx_to_vec(np.array(poseMatrix))

    def moveJ(self, qGoal):
        # qGoal is a 6 element numpy array of joint angles (radians)
        # speed is joint velocity (rad/s)
        qGoal = list(qGoal)
        print(f"MoveJ to:\n {np.degrees(qGoal).reshape((6, 1))}")
        self.c.moveJ(qGoal, 1.05, 1.4, True)

    def moveL(self, poseMatrix):
        # poseMatrix is a SE3 Object (4 x 4 Homegenous Transform) or numpy array
        # Moves tool tip pose linearly in cartesian space to goal pose (requires tool pose to be configured)
        # tool pose defined relative to the end of the gripper when closed
        poseVector = self.poseMatrixToVector(poseMatrix)
        self.c.moveL(poseVector, 0.25, 0.5, False)

    def moveStart(self):
        # Moves the arm linearly in cartesian space to home pose
        homePose = np.array([[0.99955322, -0.02418213, -0.01756664, 0.01498893],
                             [-0.01748495, 0.00358545, -0.9998407, -0.57686779],
                             [0.02424126, 0.99970114, 0.00316103, 0.05545535],
                             [0, 0, 0, 1]])
        # self.arm.move(target=homePose, move_type="l")
        self.c.moveL(homePose, 0.25, 0.5, False)

    def moveToPosition(self, position):
        # position is the numpy array similar to lines 53-56
        # vector = self.poseMatrixToVector(position)
        self.c.moveL(position, 0.25, 0.5, False)


    def openGripper(self):
        self.gripperController.openGripper()

    def closeGripper(self, width=10):
        # Computes the servo angles needed for the jaws to be width mm apart
        # Sends command over serial to the gripper to hold those angles
        dTheta = self.gripperController.distance2theta(width)
        self.gripperController.position(dTheta)

    def testRoutine(self):
        # Moves ur +1 cm in world frame z-axis then down 1 cm and then opens, closes, and opens gripper
        print("Running Test Routine")
        initPose = np.array(self.getPose())
        # print(f"TCP Pose:\n{initPose}")
        dX, dY, dZ = 0, 0, 2 / 100  # in m
        goalPose = initPose
        goalPose[2][3] += dZ
        goalPose = sm.SE3(goalPose)
        # print(f"Goal TCP Pose:\n{goalPose}")
        print("Running UR Test")
        self.moveL(sm.SE3(goalPose))
        # print(f"Final TCP Pose:\n{self.getPose()}")
        goalPose = np.array(self.getPose())
        goalPose[2][3] -= dZ
        goalPose = sm.SE3(goalPose)
        self.moveL(goalPose)
        print("Running Gripper Test")
        # print("Opening Gripper")
        self.openGripper()
        time.sleep(1)
        self.closeGripper(10)
        time.sleep(2)
        self.openGripper()