import pyrealsense2 as rs
import numpy as np
import cv2

import torch
from utils.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator, colors
from utils.serial_test import Coms
from utils.data import return_data, determine_color, determine_depth, degree
from utils.camera import switch_cameras, initialize_config
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

pipeline = rs.pipeline()

config = initialize_config(pipeline, 'f1181409')
camera = 'l515'

# Start streaming
pipeline.start(config)

try:
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
            camera = switch_cameras(pipeline, config, camera)
            if camera == 'l515':
                print("switched to l515")
            if camera == 'd435':
                print("switched to d435")
            if camera == -1:
                print("failed to switch. Falling back...")
            switch = time.time()

finally:
    pipeline.stop()

