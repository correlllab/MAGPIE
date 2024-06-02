import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import spatialmath as sm
class Block():
    def __init__(self, name, pcd, urPose):
        # :name string that is block name in PDDL
        # :pcd Open3D Point Cloud that contains only the block
        # :urPose 4x4 Matrix or SE3 Transform that is the current pose of the Nth frame of the UR5 (given by ur.getPose) when the image was taken
        self.blockPCD = pcd
        self.name = name
        # Removes outlier points by fitting block into largest cluster
        self.clusterBlockPCD() # denoises the pcd <-- this definitely seems to help with the point cloud visualization (andrea)
        self.blockAABB = self.blockPCD.get_axis_aligned_bounding_box()
        self.blockOBB = self.blockPCD.get_oriented_bounding_box(robust=True)
        self.blockAABB.color, self.blockOBB.color = [0, 0, 0], [0, 0, 0]
        self.urPose = urPose  # Pose of the Nth frame of the UR5 when the image was taken
        # get_center returns array of x, y, z coordinates
        self.camFrameCoords = np.matrix(self.blockOBB.get_center())
        self.gripperFrameCoords = self.getCenterInGripperFrame()
        self.worldFrameCoords = self.getCenterInWorld()  # Approximate coordinates in world frame

    # TODO: try this with and without the noise eliminatin to see if this is necessary
    def clusterBlockPCD(self):
        # modifies block PCD to only contain points in the largest cluster found with DBScan
        # eps found experimentally
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Error) as cm:
            # eps is radius
            # rejects points that are too small
            labels = np.array(self.blockPCD.cluster_dbscan(eps=0.005, min_points=20, print_progress=False))

        max_label = labels.max()
        colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
        colors[labels < 0] = 0
        clusters = {}
        for i in range(0, max_label + 1):
            clusters[i] = []

        for i in range(0, len(labels)):
            if labels[i] != -1:
                clusters[labels[i]].append(i)

        clusterPCDs = []  # store pcd and number of points
        for clusterLabel in clusters:
            clusterPCD = self.blockPCD.select_by_index(clusters[clusterLabel])
            clusterPCDs.append((len(clusterPCD.points), clusterPCD))

        print("clusterPCDs: ", clusterPCDs)
        clusterPCDs.sort()
        clusterPCDs.reverse()

        self.blockPCD = clusterPCDs[0][1]

    def getCenterInGripperFrame(self):
        # returns the center of the block in the gripper frame given PCD with no extrinsics applied
        R = np.array([[0, 1, 0],
                      [-1, 0, 0],
                      [0, 0, 1]])  
        # camera frame basis with respect to gripper frame
        t = np.array([0, 0.009, 0.0593])
        # Homogenous coordinates

        # TODO: Jensen, why is he choosing one over the other? Try both to test.
        # gripperFrameCoords = np.matmul(np.array(sm.SE3.Rt(R,t)),self.camFrameCoords[0:3])
        gripperFrameCoords = (sm.SE3.Rt(R, t) * sm.SE3.Trans(self.camFrameCoords[0:3])).t
        '''
        # For visualization in displayPCD
        self.real.cameraFrameTransform = np.array(camera_frame_transform)
        print(f"Camera Frame Transform:\n{self.real.cameraFrameTransform}")
        self.real.extrinsics = np.array(camera_frame_transform.inv())
        print(f"Extrinsics:\n{self.real.extrinsics}")
        '''
        return gripperFrameCoords

    # TODO: Mess with this because d seems suspicious
    def getCenterInWorld(self):
        # :urPose SE3 Transform that is the current pose of the Nth frame of the UR5 (given by ur.getPose)
        # Uses pose of the Nth frame from UR5 Interface to return the approximate center of the block in the world frame
        # may be wrong due to innacurate measurements of end-effector tool dimensions
        # CHANGE CURRENT POSE WHEN CONNECTED TO BOT
        currentPose = self.urPose  # Pose of Nth frame of UR5, SE3 Object
        # d should probably be a 3D translation but this is for testing
        d = 0.1125  # estimated distance between origin of Nth link and and center of gripper frame along urPose's z-axis (m)
        print("sm.SE3: ", sm.SE3.Tz(d))
        print("currentPose: ", currentPose)
        gripperFramePose = currentPose * sm.SE3.Tz(d)
        worldFrameCoords = (gripperFramePose * sm.SE3.Trans(self.gripperFrameCoords)).t
        return worldFrameCoords

    def get_world_frame(self, ur, z_offset=0.08):
        # given a goal position in gripper coords returns the displacements from the current pose in world coords
        xB, yB, zB = self.gripperFrameCoords
        # TODO: stop hardcoding a Z-stop position
        # maybe dynamically
        # subtract 0.165 from block position in gripper frame to account for gripper length
        # in METERS
        # zB -= 0.155
        zB -= z_offset
        currentPose = ur.get_tcp_pose()  # 4x4 homogenous transform matrix
        print(currentPose)

        R = currentPose[:3, :3]

        # xB,yB,zB here is the block position in the gripper frame which is aligned with the optoforce frame
        P_goal = np.matmul(R, np.array([xB, yB, zB]).T)  # relative position of the block in world coordinates
        print(f"P_goal:\n{P_goal}")
        dX, dY, dZ = tuple(
            P_goal)  # quantities and directions the the gripper frame should be incremented to be centered at the block
        return dX, dY, dZ

    def getWorldFrameVerticalInGripper(self, verticalDist):
        # given a displacement verticalDist along the z-axis in the world frame, determine the same displacement in the gripper frame
        # used to find the position directly above the blocks in the gripper frame
        return np.matmul(self.urPose.inv().R, (sm.SE3.Trans([0, 0, verticalDist]).t - self.urPose.t))

    def getGraspPoint(self):
        # returns the (x,y,z) coordinates in either the world or camera coordinate frame of where the gripper should be placed (depending on if extrinsics were set when creating the PCD)
        # center of front-facing axis-aligned bounding box
        x, y, z = self.blockOBB.get_center()[0:3]
        # z = self.blockAABB.get_min_bound()[2]
        return (x, y, z)

    def move(self, goalCamCoords):
        self.currentCamCoords = goalCamCoords
