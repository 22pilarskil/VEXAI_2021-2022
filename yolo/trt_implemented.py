import pyrealsense2 as rs
import numpy as np
import cv2

from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator, colors
from utils.serial_test import Coms
from utils.data import return_data, determine_color, determine_depth, degree
from utils.camera import initialize_config, switch_cameras
import time
import os

import tensorrt as trt
import trt.common
import pycuda.driver as cuda
import pycuda.autoinit
import torch

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()
    
TRT_LOGGER = trt.Logger(trt.Logger.VERBOSE)
trt.init_libnvinfer_plugins(TRT_LOGGER,'')
DTYPE_TRT = trt.float32

cameras = {
    'l515_front': 'f1181409',
    'd435_back': '048522072643',
    }
# Configure depth and color streams

pipeline = rs.pipeline()
initialize_config(pipeline, cameras['l515_front'])

comm = Coms()
try:
    comm.open()
except:
    pass

with open("best.trt", "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
    engine = runtime.deserialize_cuda_engine(f.read())
    
    with engine.create_execution_context() as context:
        try:
             while True:
                 
                

