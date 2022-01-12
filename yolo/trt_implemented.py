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

import trt_files.common as common
import pycuda.driver as cuda
import pycuda.autoinit
import torch

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()
    

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


import tensorrt as trt
TRT_LOGGER = trt.Logger(trt.Logger.VERBOSE)
trt.init_libnvinfer_plugins(TRT_LOGGER,'')
DTYPE_TRT = trt.float32
with open("newBest.engine", "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
	engine = runtime.deserialize_cuda_engine(f.read())
    
	with engine.create_execution_context() as context:
		try:
		    while True:
		        start = time.time()

		        # Wait for a coherent pair of frames: depth and color
		        frames = pipeline.wait_for_frames()
		        depth_frame = frames.get_depth_frame()
		        color_frame = frames.get_color_frame()
		        if not depth_frame or not color_frame:
		            continue

		        # Convert images to numpy arrays
		        depth_image = np.asanyarray(depth_frame.get_data())
		        color_image = np.asanyarray(color_frame.get_data())

		        depth_image = cv2.resize(depth_image, dsize=(640, 640), interpolation=cv2.INTER_AREA)
		        color_image = cv2.resize(color_image, dsize=(640, 640), interpolation=cv2.INTER_AREA)

		        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
		        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

		        depth_colormap_dim = depth_colormap.shape
		        color_colormap_dim = color_image.shape

		        color_image_t = np.transpose(color_image, [2, 0, 1])[None] / 255.0

		        #NEW STUFF -----------
		        start = time.time()
		        inputs, outputs, bindings, stream = common.allocate_buffers(engine)
		        np.copyto(inputs[0].host, color_image_t.ravel())

		        [cuda.memcpy_htod_async(inp.device, inp.host, stream) for inp in inputs]
		        inference = context.execute_async(batch_size=1, bindings=bindings, stream_handle=stream.handle)
		        [cuda.memcpy_dtoh_async(out.host, out.device, stream) for out in outputs]
		        stream.synchronize()
		 
		        
		        trt_output = [out.host for out in outputs]
        		pred = torch.tensor(np.reshape(trt_output[3], (1,25200,7)))
        		print("TRT time: {}".format(time.time()-start))
        		#NEW STUFF ---------

		        conf_thres = .3
		        pred = non_max_suppression(pred, conf_thres)

		        color_image0, depth_colormap0 = color_image, depth_colormap
		        pred = pred[0]
		        
		        # tensor([[det1],[det2]])
		        color_annotator = Annotator(color_image0, line_width=2, pil=not ascii)
		        depth_annotator = Annotator(depth_colormap0, line_width=2, pil=not ascii)
		        pred[:,:4] = scale_coords(color_image_t.shape[2:], pred[:, :4], color_image0.shape).round()

		        for i, det in enumerate(pred):
		            if(det[5] == 0): # COLOR
		                pred[i, 5] = determine_color(det, color_image)
		            else:
		                pred[i, 5] = 3
		            pred[i, 4] = determine_depth(det, depth_image) * depth_frame.get_units()

		        names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]

		        data = [0, 0]
		  
		        if int(pred.shape[0]) > 0:
		            det = return_data(pred, find="close", colors=[-1, 0, 1])

		            if len(det) > 0:
		                
		                if args.display:
		                    color_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(det[5], True))
		                    depth_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(det[5], True))
		             
		                turn_angle = degree(det)
		                if not turn_angle == None:
		                    data = [float(det[4]), float(turn_angle)]
		        
		        try:
		            print("Depth: {}, Turn angle: {}".format(data[0], data[1]))
		            comm.send("mogo", data)
		            if (comm.read("stop")): 
		                while not comm.read("continue"):
		                    print("Awaiting \"continue\" signal")
		                #switch_cameras(pipeline, config, cameras['l515_front'])

		        except:
		            try:
		                comm.open()
		            except Exception as e:
		                print(e)

		        if args.display:
		            color_image = color_annotator.result()
		            depth_colormap = depth_annotator.result()    
		            images = np.hstack((color_image, depth_colormap))
		            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		            cv2.imshow('RealSense', images)

		            print("Time elapsed: {}".format(time.time() - start))
		            cv2.waitKey(1)

		finally:
		    pipeline.stop()
