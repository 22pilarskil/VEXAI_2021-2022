import numpy as np
from utils.camera import Camera
import cv2
import time

cameras = {
    'l515_front': ('f1180887', True),
    'l515_back': ('f1181848', True),
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
            cur = 'l515_back' if cur!='l515_back' else 'l515_front'
            cam.switch_cameras(cur)
           

finally:
    cam.stop()
