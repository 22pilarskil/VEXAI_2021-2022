import cv2
from utils.camera import Camera
import time

cameras = {
    'l515_front': {
        'id':'f1180887',
        'flip':True,
        },
    'l515_back': {
        'id':'f1181848',
        'flip':True,
        }
    }

camera = 'l515_back'
cam = Camera(cameras, camera)

current_time = time.time()

while(True):
    if(time.time() - current_time >= 5):
        current_time = time.time()
        camera = 'l515_front' if camera == 'l515_back' else 'l515_back'
        cam.switch_cameras(camera)
    color_image, depth_image, color_image_t, depth_colormap, depth_frame = cam.poll_frames()
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)
