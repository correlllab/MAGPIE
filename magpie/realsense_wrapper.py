import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from PIL import Image

class RealSense():
    
    def __init__(self, zMax=0.5, voxelSize=0.001):
        self.pinholeIntrinsics = None  # set in self.takeImages()
        self.zMax = zMax  # max distance for objects in depth images (m)
        # downsample point cloud with voxel size = 1 mm (0.001 m / 0.04 in)
        self.voxelSize = voxelSize
        self.pcd = o3d.geometry.PointCloud()  # current pcd from realsense
        self.extrinsics = np.eye(4)  # extrinsic parameters of the camera frame 4 x 4 numpy array
        self.cameraFrameTransform = np.eye(4)
        self.pipe, self.config = None, None

    def initConnection(self):
        # Initializes connection to realsense, sets pipe,config values
        self.pipe = rs.pipeline()
        self.config = rs.config()

        # Getting information about the connected realsense model (device object) - D405
        pipeProfile = self.config.resolve(rs.pipeline_wrapper(self.pipe))
        device = pipeProfile.get_device()
        depth_sensor = device.first_depth_sensor()
        self.depthScale = depth_sensor.get_depth_scale()
        # print(depth_scale)

        # 1 - default, 2 - hand, 3 - high accuracy, 4 - high density, 5 - medium density
        depth_sensor.set_option(rs.option.visual_preset, 4)  # 4 corresponds to high-density option

        # Setting attributes for stream
        # Depth Stream (1280 x 720) 5 fps - D405 Sensor has max 1280 x 720
        # (Minimum z depth is between 55-70 mm)
        self.config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 5)

        # Color and Infrared D405 Streams Available (1280 x 720) 5 fps - D405 Sensor has max 1280 x 720
        self.config.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 5)

        # Starting the pipeline based on the specified configuration
        self.pipe.start(self.config)

    def getPinholeInstrinsics(self, frame):
        # frame is a subclass of pyrealsense2.video_frame (depth_frame,etc)
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        return o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx,
                                                 intrinsics.fy, intrinsics.ppx,
                                                 intrinsics.ppy)

    def takeImages(self, save=False):
        # Takes RGBD Image using Realsense
        # intrinsic and extrinsic parameters are NOT applied only in getPCD()
        # out: Open3D RGBDImage
        pipe, config = self.pipe, self.config

        frames = pipe.wait_for_frames()
        depthFrame = frames.get_depth_frame()  # pyrealsense2.depth_frame
        colorFrame = frames.get_color_frame()

        # Sets class value for intrinsic pinhole parameters
        self.pinholeInstrinsics = self.getPinholeInstrinsics(colorFrame)
        # asign extrinsics here if the camera pose is known
        # alignOperator maps depth frames to color frames
        alignOperator = rs.align(rs.stream.color)
        alignOperator.process(frames)
        alignedDepthFrame, alignedColorFrame = frames.get_depth_frame(), frames.get_color_frame()

        # unmodified rgb and z images as numpy arrays of 3 and 1 channels
        rawColorImage = np.array(alignedColorFrame.get_data())
        rawDepthImage = np.asarray(alignedDepthFrame.get_data())

        rawRGBDImage = o3d.geometry.RGBDImage.create_from_color_and_depth(
            o3d.geometry.Image(rawColorImage),
            o3d.geometry.Image(rawDepthImage.astype('uint16')),
            depth_scale=1.0 / self.depthScale,
            depth_trunc=self.zMax,
            convert_rgb_to_intensity=False)

        if save:
            subFix = str(time.time())
            np.save(f"depthImage{subFix}", rawRGBDImage.depth)
            np.save(f"colorImage{subFix}", rawRGBDImage.color)
            colorIM = Image.fromarray(rawColorImage)
            colorIM.save(f"colorImage{subFix}.jpeg")
        return rawRGBDImage

    def getPCD(self, save=False):
        # Takes images and returns a PCD and RGBD Image
        # Applies extrinsics and zMax
        # Downsamples PCD based on self.voxelSize
        # :save boolean that toggles whether to save data
        # out: tuple of (open3d point cloud (o3d.geometry.PointCloud),RGBDImage)
        rawRGBDImage = self.takeImages()
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rawRGBDImage,
            self.pinholeInstrinsics,
            project_valid_depth_only=True,
            extrinsic=self.extrinsics
        )

        # Don't downsample
        # downsampledPCD = pcd.voxel_down_sample(voxel_size=self.voxelSize)
        if save:
            subFix = time.time()
            np.save(f"colorImage{subFix}", np.array(rawRGBDImage.color))
            np.save(f"depthImage{subFix}", np.array(rawRGBDImage.depth))
            o3d.io.write_point_cloud(f"pcd{subFix}.pcd", downsampledPCD)
        return pcd, rawRGBDImage
        # return (downsampledPCD,rawRGBDImage)

    def displayImages(self, depthImg, colorImg):
        # Displays a depth and color image given the rgbdImage
        plt.subplot(1, 2, 1)
        plt.title("RealSense Color Image")
        plt.imshow(depthImg)
        plt.subplot(1, 2, 2)
        plt.title("RealSense Depth Image")
        plt.imshow(colorImg)
        plt.show()

    def displayPCD(self, pcds):
        # Displays a list of point clouds given an array of pcd's. Displays camera frame if self.extrinsics != None
        # flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]
        # pcd.transform(flip_transform)
        worldFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075, origin=[0, 0, 0])
        if (self.extrinsics is None) == False:
            cameraFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.075)
            cameraFrame.transform(self.cameraFrameTransform)
            res = [worldFrame, cameraFrame]
            res.extend(pcds)
            baseSphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.025)
            res.append(baseSphere)
            o3d.visualization.draw_geometries(res)
        else:
            res = [worldFrame].extend(pcds)
            o3d.visualization.draw_geometries(res)
    
    def display_world(self, world_pcd):
        coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometry = [coordFrame]
        geometry.append(world_pcd)
        o3d.visualization.draw_geometries(geometry)

    def display_world_nb(self, world_pcd):
        coordFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.05)
        geometry = [coordFrame]
        geometry.append(world_pcd)
        o3d.web_visualizer.draw(geometry)

    def displayStream(self):
        # streams and displays the point cloud data in open3d
        # pipe,config are stream properties set in the earlier cells
        # Streaming loop
        pipe, config = self.pipe, self.config
        vis = o3d.visualization.Visualizer()
        vis.create_window()
        framesTaken = 0
        displayedPCD = o3d.geometry.PointCloud()
        try:
            while True:
                temp = self.getPCD()[0]
                displayedPCD.points = temp.points
                displayedPCD.colors = temp.colors
                if framesTaken == 0:
                    vis.add_geometry(displayedPCD)
                    worldFrame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.25, origin=[0, 0, 0])
                    vis.add_geometry(worldFrame)
                vis.update_geometry(displayedPCD)
                framesTaken += 1
                t0 = time.time()
                vis.poll_events()
                vis.update_renderer()
                # time.sleep(5)
        except Exception as e:
            print(f"Stream Issue. Exception Raised")
            # raise(e)
        # closes window when cell is stopped (exception raised)
        finally:
            vis.destroy_window()
            # pipe.stop()

    def disconnect(self):
        self.pipe.stop()