import pyrealsense2 as rs
import numpy as np
import cv2

from utils.decorators import timer


class Camera:

    def __init__ (self, cameras, start_camera):
        self.cameras = cameras
        self.pipeline = rs.pipeline()
        self.initialize_config(self.cameras[start_camera][0])
        self.flip = self.cameras[start_camera][1]
        self.img_size = (640, 640)
        self.color_ts = 0
        self.depth_ts = 0
        self.stale_frames = 0
        self.total_frames = 0
        print(rs.context())



    def initialize_config(self, device_number):

        self.config = rs.config()
        self.device_number = device_number
        self.config.enable_device(device_number)
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device().first_depth_sensor()
        device.set_option(rs.option.min_distance, 0)
        device_product_line = str(device.get_info(rs.camera_info.product_line))
        print(device.get_info(rs.camera_info.serial_number))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(self.config)

    def print_stale_frame(self):
        print(f'Summary: Total frames: {self.total_frames}')
        print(f'Stale frames: {self.stale_frames}')


    @timer("Frame Time")
    def poll_frames(self, conn=None):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        new_color_ts = color_frame.timestamp
        new_depth_ts = depth_frame.timestamp

        print("COLOR DIFF {}".format(new_color_ts - self.color_ts))
        print("DEPTH DIFF {}".format(new_depth_ts - self.depth_ts))

        if self.color_ts == new_color_ts or self.depth_ts == new_depth_ts:
            self.stale_frames += 1
        self.total_frames += 1

        self.color_ts = new_color_ts
        self.depth_ts = new_depth_ts
        
        if not depth_frame or not color_frame:
            return None
 
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        if self.flip:
            depth_image = np.flipud(depth_image)
            color_image = np.flipud(color_image)

        depth_image = cv2.resize(depth_image, dsize=self.img_size, interpolation=cv2.INTER_AREA)
        color_image = cv2.resize(color_image, dsize=self.img_size, interpolation=cv2.INTER_AREA)

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        color_image_t = np.transpose(color_image, [2, 0, 1])[None] / 255.0

        if not conn is None:
            conn.send(color_image, depth_image, color_image_t, depth_colormap, depth_frame)
        self.print_stale_frame()
        return color_image, depth_image, color_image_t, depth_colormap, depth_frame

    

    def switch_cameras(self, camera):
        self.pipeline.stop()
        try:
            self.initialize_config(self.cameras[camera][0])
            self.flip = self.cameras[camera][1]
        except:
            self.pipeline.start(self.config)
            return False

    
    def stop(self):
        self.pipeline.stop()
