import cv2
from utils.camera import Camera
import time

cameras = {
    'l515_front': ('f1180887', True),
    'l515_back': ('f1181848', True),
    }
cam = Camera(cameras, 'l515_front')

current_time = time.time()

while(True):
    if(time.time() - current_time >= 5):
        current_time = time.time()
        Camera.switch_cameras()
        print(cam)
    color_image, depth_image, color_image_t, depth_colormap, depth_frame = Camera.poll_frames()
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)
