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
        'front': False,
        },
    'd435_front': {
        'id': '048322070458',
        'flip': True,
        'front': True,
        },
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
		pred1 = []
		for i, det in enumerate(pred):
			print("this is a det")
			turn_angle = degree(det)
			if(not math.isnan(det[6]) and not math.isnan(turn_angle)):
				depth = str(round(float(det[6]), 3))
				turn_angle = str(round(float(turn_angle), 3))
				class_id = str(int(det[5]))
				if int(class_id) == 1: contains_ring = True
				whole_str += depth + "," + turn_angle + "," + class_id + ",|"
				pred1.append([depth, turn_angle, class_id])
		#print("PRED1: " + str(pred1))

		if args.display:
			for det in pred:
				color_annotator.box_label(det[:4], f'{names[int(det[5])]} {det[4]:.2f}', color=colors(int(det[5]), True))
				depth_annotator.box_label(det[:4], f'{names[int(det[5])]} {det[4]:.2f}', color=colors(int(det[5]), True))
			color_image, depth_colormap = color_annotator.result(), depth_annotator.result()
			images = np.hstack((color_image, depth_colormap))
			cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
			cv2.imshow('RealSense', images)
			cv2.waitKey(1)

	
		if True:
			print(whole_str)
			
			comm.send("whole_data", whole_str)

			msg = False
			while not msg:
				print("waiting")
				msg = comm.read([str(id) for id in range(1, 37)] + ["x","y","z","@"])

			
			dict_ = {}
			front = cam.cameras[cam.name]['front']
			robot_x = float(msg["x"])
			robot_y = float(msg["y"])
			robot_heading = 90 - float(msg["z"]) if front else 270 - float(msg["z"])
			robot_coords = displayer.robot_coords(robot_x, robot_y)
			in_view = displayer.boxes_inside(robot_heading, robot_coords[0], robot_coords[1])
			print(in_view)
			new_det = displayer
			print("P1:" + str(pred1))

			det_box = displayer.det_to_boxes(robot_x, robot_y, robot_heading, pred1)
			print("DETBOX: " + str(det_box))
			box_numrings = {}
			box_nummogos = {}
			for det in det_box:
				det = [int(det[0]), int(det[1])]
				box_id = det[0]
				if(det[1]==0):
					if not box_id in box_numrings: box_numrings[box_id] = 1
					else: box_numrings[box_id] += 1
				if(det[1]== 1):
					if not box_id in box_nummogos: box_nummogos[box_id] = 1
					else: box_nummogos[box_id] +=1
			full_dict = {}
			print("BOXNUMRINGS: " +str(box_numrings))
			print("BOXNUMMOGOS: " +str(box_nummogos))
			print("CHECK: " + str(in_view))
			for box in range(1,37):
				full_dict[box] = [0,0]
				if(in_view[box]):
					rings = 0
					mogos = 0
					if box in box_numrings: rings = box_numrings[box]
					if box in box_nummogos: mogos = box_nummogos[box]
					
					full_dict[box] = [mogos, rings]
			print("DICTIONARY: " +str(full_dict))
			displayer.runner(full_dict, robot_coords[0], robot_coords[1], robot_heading)
				

			if "stop" in msg:
				bcolors.print("STOP", "green")
		
			if(msg==False):
				comm.wait(["continue"])
				print("HERE")
			
			
			#comm.send("fps", time.time())
		 
		#except Exception as e:
		#	bcolors.print(str(e), "blue")
		#	comm.open()
finally:
	cam.stop()
