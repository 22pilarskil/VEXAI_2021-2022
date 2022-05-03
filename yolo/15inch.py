import numpy as np
import cv2
import time
import argparse
import math
import matplotlib.pyplot as plt
import torch
import random
from utils.animation import Display
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_depth, degree, sort_distance, quicksort
from utils.camera import Camera
from utils.models import Model
from sklearn.cluster import DBSCAN
from utils.decorators import bcolors

parser = argparse.ArgumentParser()
parser.add_argument("--camera", metavar="camera", type=str, default="l515_back")
parser.add_argument("--display", metavar = "display",type=bool,default=True)



args = parser.parse_args()
whole_str = ""
names = ["mogo", "ring"]
model = Model("models/weights/best_yolov5n.engine")
conf_thres = .4

cameras = {
    'l515_back': {
        'id': 'f1181409',
        'flip': True,
        }
    }

cam = Camera(cameras, args.camera)

comm = Coms()
displayer = Display()
temp_msg = ""

try:
	while(True):
		data = cam.poll_frames()
		if data is None:
			continue
		color_image, depth_image, color_image_t, depth_colormap, depth_frame = data
		pred = model.predict(color_image_t, color_image.shape, conf_thres=conf_thres)
		pred = torch.column_stack((pred, torch.ones(len(pred))))

		for i, det in enumerate(pred):
			pred[i, 6] = determine_depth(det, depth_image) * depth_frame.get_units()
		try:
			pred = torch.stack(quicksort(pred))
		except:
			pass
		color_annotator = Annotator(color_image, line_width=2, pil=not ascii)
		depth_annotator = Annotator(depth_colormap, line_width=2, pil=not ascii)


		contains_ring = False
		whole_str = ""
		for i, det in enumerate(pred):
			turn_angle = degree(det)
			if(not math.isnan(det[6]) and not math.isnan(turn_angle)):
				depth = str(round(float(det[6]), 3))
				turn_angle = str(round(float(turn_angle), 3))
				class_id = str(int(det[5]))
				if int(class_id) == 1: contains_ring = True
				whole_str += depth + "," + turn_angle + "," + class_id + ",|"

		if args.display:
			for det in pred:
				color_annotator.box_label(det[:4], f'{names[int(det[5])]} {det[4]:.2f}', color=colors(int(det[5]), True))
				depth_annotator.box_label(det[:4], f'{names[int(det[5])]} {det[4]:.2f}', color=colors(int(det[5]), True))
			color_image, depth_colormap = color_annotator.result(), depth_annotator.result()
			images = np.hstack((color_image, depth_colormap))
			cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
			cv2.imshow('RealSense', images)
			cv2.waitKey(1)
		try:
			print(whole_str)
			
			comm.send("whole_data", whole_str)

			msg = False
			while not msg:
				print("waiting")
				msg = comm.read(["1", "2","3","4","5","6","7","8","9","10","11","12","13","14","15","16","17","18","19","20","21","22","23","24","25","26","27","28","29","30","31","32","33","34","35","36", "@"])

			
			dict_ = {}
			print("RECEIVED: " + str(msg))
			for x in range(1,37):
				if str(x) in msg:
					temp = msg[str(x)].split()
					ring = int(temp[1])
					mogo = int(temp[0])
					dict_[x] = [mogo, ring]
						
						
				else: 
					dict_[x] = [0,0]
			print("DICTIONARY: " + str(dict_))
			displayer.runner(dict_)

			if "stop" in msg:
				bcolors.print("STOP", "green")
		
			if(msg==False):
				comm.wait(["continue"])
				print("HERE")
			
			
			#comm.send("fps", time.time())
		 
		except Exception as e:
			bcolors.print(str(e), "blue")
			comm.open()
finally:
	cam.stop()
