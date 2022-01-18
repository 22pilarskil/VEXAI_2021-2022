import numpy as np
from utils.camera import Camera
import cv2
import time

cameras = {
    'l515_front': 'f1181409',
    'd435_back': '048522072643',
    }
cam = Camera(cameras, 'l515_front')


try:
    while True:
        start = time.time()

        data = cam.poll_frames()
        if data is None: 
            print("failed")
            continue
        color_image, depth_image, color_image_t, depth_colormap, depth_frame = data

        print("Time elapsed: {}".format(time.time() - start))

        images = np.hstack((color_image, depth_colormap))
        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        cv2.waitKey(1)
           

finally:
    cam.stop()
