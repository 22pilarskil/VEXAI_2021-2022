import cv2
from utils.camera import Camera
import time
import argparse

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

parser = argparse.ArgumentParser()
parser.add_argument("--camera", metavar="camera", type=str, default="l515_back")

args = parser.parse_args()


camera = args.camera
cam = Camera(cameras, camera)

current_time = time.time()

while(True):
    if(time.time() - current_time >= 1):
        current_time = time.time()
        camera = 'l515_front' if camera == 'l515_back' else 'l515_back'
        cam.switch_cameras(camera)
    color_image, depth_image, color_image_t, depth_colormap, depth_frame = cam.poll_frames()
    cv2.imshow('RealSense', color_image)
    cv2.waitKey(1)
