from typing import List

from ultralytics import YOLO
from PIL import Image
import numpy as np
import cv2
import open3d as o3d
import Block as bl
import spatialmath as sm


class TorchImage:
    def __init__(self, pose, ci, di):
        self.pose = pose
        self.colorImage = ci
        self.depthImage = di

class ImagePacket:
    def __init__(
            self,
            torch_image: TorchImage,
            red_pcd: o3d.geometry.PointCloud,
            yellow_pcd: o3d.geometry.PointCloud,
            blue_pcd: o3d.geometry.PointCloud,
            red_block: bl.Block,
            yellow_block: bl.Block,
            blue_block: bl.Block,

    ):
        self.torch_image = torch_image
        self.red_pcd = red_pcd
        self.yellow_pcd = yellow_pcd
        self.blue_pcd = blue_pcd
        self.red_block = red_block
        self.yellow_block = yellow_block
        self.blue_block = blue_block

    def get_all_pcds(self) -> List[o3d.geometry.PointCloud]:
        return [self.red_pcd, self.yellow_pcd, self.blue_pcd]

    def get_all_blocks(self) -> List[bl.Block]:
        return [self.red_block, self.yellow_block, self.blue_block]

    def get_all_block_world_coords(self, ur):
        return [self.red_block.get_world_frame(ur), self.yellow_block.get_world_frame(ur), self.blue_block.get_world_frame(ur)] # TODO: test that this method is correct

class ObjectDetection():
    # This class creates a RealSense Object, takes images and returns Open3D point clouds corresponding to blocks
    # Extrinsics of RealSense Object are no longer used here so interfacing with the Realsense could be done outside this class for decoupling
    # Additionally, migration of world and gripper frame position to block objects means moveRelative can also be removed
    def __init__(self, RealSense, robot_model, moveRelative=True):
        # :RealSense - RealSense object
        # :moveRelative boolean - True if the gripper moves to block positions in the camera frame, false if moving to world frame positions (via rtb_model)
        # robot_model is object of type RTB_Model
        self.real = RealSense
        '''
        # self.real.initConnection()
        if moveRelative:
            t = np.array([0,9,59.3]) / 1000
            R = np.array([[0,1,0],[-1,0,0],[0,0,1]])
            camera_frame_transform = sm.SE3.Rt(R,t)
            self.real.cameraFrameTransform = np.array(camera_frame_transform)
            # print(f"Camera Frame Transform:\n{self.real.cameraFrameTransform}")
            self.real.extrinsics = np.array(camera_frame_transform.inv())
            # print(f"Extrinsics:\n{self.real.extrinsics}")
        else:
            T_C = robot_model.getCameraFrame()
            print(T_C)
            self.real.cameraFrameTransform = np.array(T_C)
            self.real.extrinsics = np.array(T_C.inv())
        '''
        # Load the model into memory
        # Trained on yolov8l-seg for 200 epochs
        # yolo models compared https://docs.ultralytics.com/tasks/segment/
        self.model = YOLO('yolov8l-seg.pt')

    def getSegmentationMask(self, result, className):
        # :result ultralytics.result
        # :className string corresponding to label in trained YOLO model
        # Here, className should be in {'Red','Yellow','Blue'}
        # Returns 1st instance of the class as binary numpy array and None if the class is not present
        classList = list(np.array(result.boxes.cls.cpu()))
        for i in range(0, len(classList)):
            predictedClassName = result.names[classList[i]]
            if predictedClassName == className:
                mask = result.masks.data[i].numpy()  # (384,640)
                # Resize mask to original imae size
                scaledMask = cv2.resize(mask, (result.orig_shape[1], result.orig_shape[0]))
                return scaledMask
        return None

    def get_pcds_of_all_images(self, images: List[TorchImage]) -> List[ImagePacket]:
        '''
        Params: List of Pytorch Model Images
        Returns List of Enriched ImagePacket's that contain all of the pcd's for each block
        '''
        image_packets = []
        for image in images:
            pilImage = Image.fromarray(np.array(image.colorImage))
            result = self.model.predict(pilImage, conf=0.6, save=True)[0]
            redMask = self.getSegmentationMask(result, 'Red')
            yellowMask = self.getSegmentationMask(result, 'Yellow')
            blueMask = self.getSegmentationMask(result, 'Blue')

            redDepthImage = np.multiply(image.depthImage, redMask.astype(int)).astype('float32')
            yellowDepthImage = np.multiply(image.depthImage, yellowMask.astype(int)).astype('float32')
            blueDepthImage = np.multiply(image.depthImage, blueMask.astype(int)).astype('float32')

            # SEGMENT PCD INTO RED,YELLOW,BLUE BLOCKS

            redPCD = o3d.geometry.PointCloud.create_from_depth_image(
                depth=o3d.geometry.Image(redDepthImage),
                intrinsic=self.real.pinholeInstrinsics,
                depth_scale=1000.0,
                depth_trunc=1000.0,
                stride=1,
                project_valid_depth_only=True)

            yellowPCD = o3d.geometry.PointCloud.create_from_depth_image(
                depth=o3d.geometry.Image(yellowDepthImage),
                intrinsic=self.real.pinholeInstrinsics,
                depth_scale=1000.0,
                depth_trunc=1000.0,
                stride=1,
                project_valid_depth_only=True)

            bluePCD = o3d.geometry.PointCloud.create_from_depth_image(
                depth=o3d.geometry.Image(blueDepthImage),
                intrinsic=self.real.pinholeInstrinsics,
                depth_scale=1000.0,  # FYI: previously this scale was set to 1
                depth_trunc=1000.0,
                stride=1,
                project_valid_depth_only=True)  # TODO: try this line for both off and on with all PCDs

            '''
            # Downsample point cloud's based on realsense voxel_size parameter
            redPCD = redPCD.voxel_down_sample(voxel_size=self.real.voxelSize)
            yellowPCD = yellowPCD.voxel_down_sample(voxel_size=self.real.voxelSize)
            bluePCD = bluePCD.voxel_down_sample(voxel_size=self.real.voxelSize)
            '''
            redPCD.paint_uniform_color([1, 0, 0])
            yellowPCD.paint_uniform_color([1, 1, 0])
            bluePCD.paint_uniform_color([0, 0, 1])

            redBlock = bl.Block("redBlock", redPCD, image.pose)
            yellowBlock = bl.Block("yellowBlock", yellowPCD, image.pose)
            blueBlock = bl.Block("blueBlock", bluePCD, image.pose)

            image_packets.append(ImagePacket(image, redPCD, yellowPCD, bluePCD, redBlock, yellowBlock, blueBlock))
        return image_packets

    def getBlocksFromImages(self, images: List[TorchImage], ur):
        # :colorImage 3-channel rgb image as numpy array
        # :depthImage 1-channel of measurements in z-axis as numpy array
        # :display boolean that toggles whether masks should be shown with color image
        # :urPose 4x4 numpy array or SE3 transform that is the pose of the Nth frame when the images were taken
        # colorImage,depthImage = RGBD_Image.color,RGBD_Image.depth
        # Returns a tuple of (RedPCD,yellowPCD,bluePCD) corresponding to each block class

        # Detects and segments classes using trained yolov8l-seg model
        # Inference step, only return instances with confidence > 0.6

        image_packets: List[ImagePacket] = self.get_pcds_of_all_images(images)

        # avg world frame coords for all blocks
        redBlock = []
        yellowBlock = []
        blueBlock = []
        for packet in image_packets:
            coords = packet.get_all_block_world_coords(ur) # returned as [red, yellow, blue]
            redBlock.append(coords[0])
            yellowBlock.append(coords[1])
            blueBlock.append(coords[2])

        redAvg = np.mean(redBlock, axis=0)
        yellowAvg = np.mean(yellowBlock, axis=0)
        blueAvg = np.mean(blueBlock, axis=0)

        # o3d.visualization.draw([redPCD,yellowPCD,bluePCD])
        # o3d.visualization.draw_geometries([redPCD,yellowPCD,bluePCD])

        return (redAvg, yellowAvg, blueAvg)
        # return (redPCD,yellowPCD,bluePCD)

    def displayWorld(self, worldPCD, blocks):
        coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometry = [coordFrame]
        geometry.append(worldPCD)
        for block in blocks:
            geometry.append(block.blockPCD)
            geometry.append(block.blockOBB)
            # why are we creating a sphere here. 
            # TODO: try create_box method instead of create_sphere
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.0035)
            # TODO: figure out what transformations are happening here and would it be better to use Magpie's homogoneous matrix method?
            sphere.transform(np.array(sm.SE3.Trans(block.camFrameCoords))) # Apply transformation (4x4 matrix) to the geometry coordinates
            geometry.extend([sphere])
            '''
            print(f"{block.name}")
            deltas = ["dx","dy","dz"]
            for i in range(0,len(block.robotCoords)):
                print(f"{deltas[i]}: {block.robotCoords[i]}")

            print(f"{block.name}\nCam Coordinates: {block.camCoords}")
            '''
            # print(f"Robot Coordinates: {block.robotCoords}")
        o3d.visualization.draw_geometries(geometry)
