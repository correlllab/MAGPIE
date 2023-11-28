import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import spatialmath as sm

########## Simple Block ############################################################################

class SimpleBlock:
    """ Use this to ground the RYB blocks """
    def __init__( self, name, pcd, pose ):
        self.name = name
        self.pcd  = pcd
        self.pose = pose



########## Complex Block ###########################################################################

class Block:
    
    def __init__(self, name, pcd, urPose):
        # :name string that is block name in PDDL
        # :pcd Open3D Point Cloud that contains only the block
        # :urPose 4x4 Matrix or SE3 Transform that is the current pose of the Nth frame of the UR5 (given by ur.getPose) when the image was taken
        self.blockPCD = pcd
        print( "PCD:", self.blockPCD, type( self.blockPCD ) )
        self.name = name
        # Removes outlier points by fitting block into largest cluster
        self.clusterBlockPCD()
        self.blockAABB = self.blockPCD.get_axis_aligned_bounding_box()
        self.blockOBB = self.blockPCD.get_oriented_bounding_box()
        self.blockAABB.color, self.blockOBB.color = [0, 0, 0], [0, 0, 0]
        self.urPose = urPose  # Pose of the Nth frame of the UR5 when the image was taken
        x, y = self.blockAABB.get_center()[0:2]
        # due to convex hull outliers are included when mask is off. Use min bound rather than center
        zMin = self.blockAABB.get_min_bound()[2]
        self.camFrameCoords = np.matrix([x, y, zMin])
        self.gripperFrameCoords = self.getCenterInGripperFrame()
        self.worldFrameCoords = self.getCenterInWorld()  # Approximate coordinates in world frame

    def clusterBlockPCD(self):
        # modifies block PCD to only contain points in the largest cluster found with DBScan
        # eps found experimentally
        with o3d.utility.VerbosityContextManager(
                o3d.utility.VerbosityLevel.Error) as cm:
            # eps is radius
            # rejects points that are too small
            # labels = np.array(self.blockPCD.cluster_dbscan(eps=0.005, min_points=20, print_progress=False))
            labels = np.array(self.blockPCD.cluster_dbscan(eps=0.005, min_points=10, print_progress=False))

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

        print(clusterPCDs)

        def myFunc(e):
            """ https://www.w3schools.com/python/ref_list_sort.asp """
            # return e[0]
            return len(e)
        
        clusterPCDs.sort( key = myFunc )
        clusterPCDs.reverse()

        print( f"Found {len(clusterPCDs)} clouds!" )

        print( type( clusterPCDs[0] ) )
        
        self.blockPCD = clusterPCDs[0][1]
        # self.blockPCD = clusterPCDs[0]

    def getCenterInGripperFrame(self):
        # returns the center of the block in the gripper frame given PCD with no extrinsics applied
        R = np.array([[0, 1, 0],
                      [-1, 0, 0],
                      [0, 0, 1]])  # camera frame basis with respect to gripper frame
        t = np.array([0, 9, 59.3]) / 1000  # camera frame origin with respect to gripper frame (mm)
        # Homogenous coordinates
        # gripperFrameCoords = np.matmul(np.array(sm.SE3.Rt(R,t)),self.cameraFrameCoords[0:3])
        gripperFrameCoords = (sm.SE3.Rt(R, t) * sm.SE3.Trans(self.camFrameCoords[0:3])).t
        '''
        # For visualization in displayPCD
        self.real.cameraFrameTransform = np.array(camera_frame_transform)
        print(f"Camera Frame Transform:\n{self.real.cameraFrameTransform}")
        self.real.extrinsics = np.array(camera_frame_transform.inv())
        print(f"Extrinsics:\n{self.real.extrinsics}")
        '''

        return gripperFrameCoords

    def getCenterInWorld(self):
        # :urPose SE3 Transform that is the current pose of the Nth frame of the UR5 (given by ur.getPose)
        # Uses pose of the Nth frame from UR5 Interface to return the approximate center of the block in the world frame
        # may be wrong due to innacurate measurements of end-effector tool dimensions
        # CHANGE CURRENT POSE WHEN CONNECTED TO BOT
        currentPose = self.urPose  # Pose of Nth frame of UR5, SE3 Object
        # d should probably be a 3D translation but this is for testing
        d = 0.1125  # estimated distance between origin of Nth link and and center of gripper frame along urPose's z-axis (m)
        gripperFramePose = currentPose * sm.SE3.Tz(d)
        # worldFrameCoords = (gripperFramePose * sm.SE3.Trans(self.gripperFrameCoords)).t
        worldFrameCoords = (gripperFramePose * sm.SE3.Trans(self.gripperFrameCoords))
        return worldFrameCoords

    def getWorldFrameVerticalInGripper(self, verticalDist):
        # given a displacement verticalDist along the z-axis in the world frame, determine the same displacement in the gripper frame
        # used to find the position directly above the blocks in the gripper frame
        return np.matmul(self.urPose.inv().R, (sm.SE3.Trans([0, 0, verticalDist]).t - self.urPose.t))

    def getGraspPoint(self):
        # returns the (x,y,z) coordinates in either the world or camera coordinate frame of where the gripper should be placed (depending on if extrinsics were set when creating the PCD)
        # center of front-facing axis-aligned bounding box
        x, y, z = self.blockAABB.get_center()[0:3]
        # z = self.blockAABB.get_min_bound()[2]
        return (x, y, z)

    def move(self, goalCamCoords):
        self.currentCamCoords = goalCamCoords
