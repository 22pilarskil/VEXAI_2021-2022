import pyrealsense2 as rs
import numpy as np
import cv2

import torch
from utils.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator, colors
from utils.serial_test import Coms
from utils.data import return_data, determine_color, determine_depth, degree
import time
import os, sys

PATH = os.getcwd() + "/models/best.pt"
do_depth_ring = False

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()
    
        
model = attempt_load(PATH)
device = torch.device("cuda" if torch.cuda.is_available() else 'cpu')
model.to(device)
names = model.module.names if hasattr(model, 'module') else model.names

# Configure depth and color streams
pipeline_l515 = rs.pipeline()
pipeline_d435 = rs.pipeline()

config_l515 = rs.config()
config_l515.enable_device('f1181409')

config_d435 = rs.config()
config_d435.enable_device('048522072643')

# Get device product line for setting a supporting resolution
pipeline_wrapper_l515 = rs.pipeline_wrapper(pipeline_l515)
pipeline_wrapper_d435 = rs.pipeline_wrapper(pipeline_d435)
pipeline_profile_l515 = config_l515.resolve(pipeline_wrapper_l515)
pipeline_profile_d435 = config_d435.resolve(pipeline_wrapper_d435)
device_l515 = pipeline_profile_l515.get_device()
device_d435 = pipeline_profile_d435.get_device()
device_product_line_l515 = str(device_l515.get_info(rs.camera_info.product_line))
device_product_line_d435 = str(device_d435.get_info(rs.camera_info.product_line))

config_l515.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config_d435.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line_l515 == 'L500':
    config_l515.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config_l515.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)


if device_product_line_d435 == 'L500':
    config_d435.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config_d435.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline_l515.start(config_l515)
pipeline_d435.start(config_d435)

try:
    pipeline = pipeline_l515
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    switch = time.time()
    while True:
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        color_colormap_dim = color_image.shape

        color_image_t = torch.cuda.FloatTensor(color_image)
        color_image_t = torch.moveaxis(color_image_t, 2, 0)[None] / 255.0

        cv2.imshow('RealSense', color_image)
        cv2.waitKey(1)

        if time.time()-switch >= 10:
            print("switching")
            if pipeline == pipeline_d435:
                print("switched to l515")
                pipeline = pipeline_l515
            elif pipeline == pipeline_l515:
                print("switched to d435")
                pipeline = pipeline_d435
            switch = time.time()

finally:
    pipeline.stop()

