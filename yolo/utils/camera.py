import pyrealsense2 as rs
import numpy as np
import cv2

class Camera:

    def __init__ (self, cameras, start_camera):
        self.cameras = cameras
        self.pipeline = rs.pipeline()
        self.initialize_config(self.cameras[start_camera])
        self.img_size = (640, 640)

    def initialize_config(self, device_number):

        self.config = rs.config()
        self.config.enable_device(device_number)
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = self.config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device().first_depth_sensor()
        device.set_option(rs.option.min_distance, 0)
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        self.pipeline.start(self.config)

    def poll_frames(self):
        frames = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None
 
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        depth_image = cv2.resize(depth_image, dsize=self.img_size, interpolation=cv2.INTER_AREA)
        color_image = cv2.resize(color_image, dsize=self.img_size, interpolation=cv2.INTER_AREA)

        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        color_image_t = np.transpose(color_image, [2, 0, 1])[None] / 255.0

        return color_image, depth_image, color_image_t, depth_colormap, depth_frame
    

    def switch_cameras(self, device_number):
        self.pipeline.stop()
        try:
            self.initialize_config(device_number)
        except:
            self.pipeline.start(self.config)
            return -1

    def stop(self):
        self.pipeline.stop()
