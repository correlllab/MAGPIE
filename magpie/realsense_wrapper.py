import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import matplotlib.pyplot as plt
from PIL import Image
import time
import subprocess
import os
import asyncio
import threading

def poll_devices():
    ctx = rs.context()
    devices = ctx.query_devices()
    models = [d.get_info(rs.camera_info.name).split(' ')[-1] for d in devices]
    serial = [d.get_info(rs.camera_info.serial_number) for d in devices]
    info = {i:j for i,j in zip(models, serial)}
    return info

class RealSense():
    def __init__(self, w=1280, h=720, zMax=0.5, voxelSize=0.001, fps=5, device_serial=None, device_name='D405'):
        self.pinholeIntrinsics = None  # set in self.takeImages()
        self.zMax = zMax  # max distance for objects in depth images (m)
        # downsample point cloud with voxel size = 1 mm (0.001 m / 0.04 in)
        self.voxelSize = voxelSize
        self.pcd = o3d.geometry.PointCloud()  # current pcd from realsense
        self.extrinsics = np.eye(4)  # extrinsic parameters of the camera frame 4 x 4 numpy array
        self.cameraFrameTransform = np.eye(4)
        self.pipe, self.config, self.device = None, None, None
        self.recording = False
        self.recording_task = None
        self.fps = fps # fps can only be: 5, 15, 30, 60, 90
        self.device_name = device_name
        if self.device_name is not None:
            try:
                self.device_serial = poll_devices()[self.device_name]
            except KeyError:
                print("Device not found. Please check device name. Default is D405")
                self.device_serial = None
        else:
            self.device_serial = device_serial
        self.w = w
        self.h = h
        self.buffer_dict = {}

    def initConnection(self, device_serial=None, enable_depth=True, enable_color=True):
        # Initializes connection to realsense, sets pipe,config values
        self.pipe = rs.pipeline()
        self.config = rs.config()

        # enable specific device, used if multiple devices connected
        # THIS CALL MUST HAPPEN BEFORE EVERYTHING ELSE
        # Especially before calling get_device() on config.resolve(...), duh...
        if self.device_serial is None and device_serial is not None:
            self.device_serial = device_serial
        self.config.enable_device(self.device_serial)

        # Getting information about the connected realsense model (device object) - D405
        pipeProfile = self.config.resolve(rs.pipeline_wrapper(self.pipe))
        self.device = device = pipeProfile.get_device()
        depth_sensor = device.first_depth_sensor()
        ''' trying to improve FPS, not needed anymore
        depth_sensor.set_option(rs.option.enable_auto_exposure, False)
        set exposure
        depth_sensor.set_option(rs.option.exposure, 400)

        if self.device_name != 'D405':
            color_sensor = device.first_color_sensor()
            color_sensor.set_option(rs.option.enable_auto_exposure, False)
            color_sensor.set_option(rs.option.exposure, 400)
        '''
        self.depthScale = depth_sensor.get_depth_scale()
        # print(depth_scale)

        # 1 - default, 2 - hand, 3 - high accuracy, 4 - high density, 5 - medium density
        depth_sensor.set_option(rs.option.visual_preset, 4)  # 4 corresponds to high-density option

        # Setting attributes for stream
        # Depth Stream (1280 x 720) 5 fps - D405 Sensor has max 1280 x 720
        # (Minimum z depth is between 55-70 mm)
        if enable_depth:
            self.config.enable_stream(rs.stream.depth, self.w, self.h, rs.format.z16, self.fps)

        # Color and Infrared D405 Streams Available (1280 x 720) 5 fps - D405 Sensor has max 1280 x 720
        if enable_color:
            self.config.enable_stream(rs.stream.color, self.w, self.h, rs.format.rgb8, self.fps)

        # Starting the pipeline based on the specified configuration
        p = self.pipe.start(self.config)

    def getPinholeInstrinsics(self, frame):
        # frame is a subclass of pyrealsense2.video_frame (depth_frame,etc)
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        return o3d.camera.PinholeCameraIntrinsic(intrinsics.width, intrinsics.height, intrinsics.fx,
                                                 intrinsics.fy, intrinsics.ppx,
                                                 intrinsics.ppy)

    def write_buffer(self):
        for path, im in self.buffer_dict.items():
            colorIM = Image.fromarray(im)
            colorIM.save(path)
        self.buffer_dict = {}

    def flush_buffer(self, t=3):
        start = time.time()
        while time.time() - start < t:
            frames = self.pipe.wait_for_frames()

    async def take_image(self, save=False, filepath="", buffer=False):
        # Takes RGBD Image using Realsense
        # intrinsic and extrinsic parameters are NOT applied only in getPCD()
        # out: Open3D RGBDImage
        pipe, config = self.pipe, self.config

        frames = pipe.wait_for_frames()
        colorFrame = frames.get_color_frame()
        rawColorImage = np.asanyarray(colorFrame.get_data()).copy()
        timestamp = frames.get_timestamp()
        # get sensor timestamp
        ts = frames.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp)
        # move ts decimal 3 places to the right
        timestamp = timestamp / 1000

        if save:
            # subFix = str(time.time())
            subFix = str(timestamp)
            if buffer:
                self.buffer_dict[f"{filepath}{subFix}.jpeg"] = rawColorImage

        return rawColorImage

    def take_image_blocking(self, save=False, filepath="", buffer=False):
        # Takes RGBD Image using Realsense
        # intrinsic and extrinsic parameters are NOT applied only in getPCD()
        # out: Open3D RGBDImage
        pipe, config = self.pipe, self.config

        frames = pipe.wait_for_frames()
        colorFrame = frames.get_color_frame()
        rawColorImage = np.asanyarray(colorFrame.get_data()).copy()
        timestamp = frames.get_timestamp()
        # get sensor timestamp
        ts = frames.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp)
        # move ts decimal 3 places to the right
        timestamp = timestamp / 1000

        if save:
            # subFix = str(time.time())
            subFix = str(timestamp)
            if buffer:
                self.buffer_dict[f"{filepath}{subFix}.jpeg"] = rawColorImage

        return rawColorImage


    async def _record_images(self, filepath=""):
        # records images to a specified filepath
        try:
            while self.recording:
                await self.take_image(save=True, filepath=filepath, buffer=True)
                await asyncio.sleep(0.01)
        except asyncio.CancelledError:
            print(f"{self.device_name} Recording Task Cancelled")

    def begin_record(self, filepath=""):
        if not self.recording:
            self.recording = True
            self.recording_task = asyncio.create_task(self._record_images(filepath=filepath))
            print(f"{self.device_name} Recording Started")

    async def stop_record(self, flush_time=2):
        if self.recording:
            self.recording = False
            if self.recording_task is not None:
                self.recording_task.cancel()
                try:
                    await self.recording_task
                except asyncio.CancelledError:
                    pass
                print(f"{self.device_name} Recording Stopped")
            self.write_buffer()
            print(f"{self.device_name} Buffer written")
            # self.flush_buffer(t=flush_time)
            # print("Buffer flushed")
        else:
            print("Recording inactive")

    def takeImages(self, save=False, filepath=""):
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
            np.save(f"{filepath}depthImage{subFix}", rawRGBDImage.depth)
            np.save(f"{filepath}colorImage{subFix}", rawRGBDImage.color)
            colorIM = Image.fromarray(rawColorImage)
            colorIM.save(f"{filepath}colorImage{subFix}.jpeg")
        return rawRGBDImage

    def getPCD(self, save=False, adjust_extrinsics=False):
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

        if adjust_extrinsics:
            # create rotation matrix of -pi/2 about z-axis
            rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
            tmat_gripper = np.array([[1, 0, 0, -1.15 / 100],
                            [0, 1, 0, 1.3 / 100],
                            [0, 0, 1, (309.63 - 195.0) / 1000],
                            [0, 0, 0, 1]])
            pcd.rotate(rot) # account for camera orientation, which is -pi/2 about z-axis relative to ur5 wrist
            pcd.transform(tmat_gripper) # account for camera position relative to ur5 wrist

        return pcd, rawRGBDImage
        # return (downsampledPCD,rawRGBDImage)

    def apply_extrinsics(self, pcd):
        # Applies extrinsics to the camera frame, returning pcd back to wrist frame
        # pose is a 4x4 numpy array
        # pcd is an open3d point cloud
        # create rotation matrix of -pi/2 about z-axis
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
        tmat_gripper = np.array([[1, 0, 0, -1.15 / 100],
                        [0, 1, 0, 1.3 / 100],
                        [0, 0, 1, (309.63 - 195.0) / 1000],
                        [0, 0, 0, 1]])
        pcd.rotate(rot) # account for camera orientation, which is -pi/2 about z-axis relative to ur5 wrist
        pcd.transform(tmat_gripper) # account for camera position relative to ur5 wrist
        return pcd


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