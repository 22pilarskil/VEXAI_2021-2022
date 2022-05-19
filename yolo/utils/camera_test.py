"""
import numpy as np
import cv2
import time

from decorators import timer, bcolors


class Camera:

    def __init__ (self, cameras, name):

        self.cameras = cameras
        self.name = name

        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.initialize_config(self.cameras[self.name]['id'])

        self.img_size = (640, 640)
        self.align = rs.align(rs.stream.color)
 


    def initialize_config(self, device_number):

        self.config.enable_device(device_number)
"""
import pyrealsense2 as rs
config = rs.context().devices
for device in config:
    print(device.get_info(rs.camera_info.serial_number))


import numpy as np
from camera import Camera
import cv2
import time

cameras = {
    'd435_front': ('017322071832', True),
    'l515_back': ('f1181409', True),
    }
cam = Camera(cameras, 'l515_back')

cur = 'l515_back'

start = time.time() - 4
try:
    while True:
     

        data = cam.poll_frames()
        if data is None: 
            print("failed")
            continue
        color_image, depth_image, color_image_t, depth_colormap, depth_frame = data

        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
        if time.time() - start > 5:
            start = time.time()
            cur = 'l515_back' if cur!='l515_back' else 'd435_front'
            cam.switch_cameras(cur)
           

finally:
    cam.stop()
