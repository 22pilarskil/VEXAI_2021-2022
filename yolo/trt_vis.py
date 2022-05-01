import numpy as np
import cv2
import time
import argparse
import math

import torch
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_depth, degree, sort_distance, quicksort
from utils.camera import Camera
from utils.models import Model
from sklearn.cluster import DBSCAN
from utils.decorators import bcolors

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=bool, default=True)
parser.add_argument("--camera", metavar="camera", type=str, default="l515_back")



args = parser.parse_args()
whole_str = ""
names = ["mogo", "ring"]
model = Model("models/weights/best_yolov5n.engine")
conf_thres = .4

cameras = {
    'l515_front': {
        'id': 'f1181409',
        'flip': False,
        },
    'l515_back': {
        'id': 'f1181409',
        'flip': False,
        },
    }

cam = Camera(cameras, args.camera)

comm = Coms()

try:
    while True:
        start = time.time()
        try:
            data = cam.poll_frames()
        except Exception as e:
            bcolors.print(str(e), "blue")
            continue
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

        print("-------------")

        print("Time elapsed: {}".format(time.time() - start))

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
            msg = comm.read(["camera", "stop"])
            print(msg)
            if "camera" in msg:
                bcolors.print("Switching to " + str(msg["camera"]), "green")
                if not cam.name == msg["camera"]: print(cam.switch_cameras(msg["camera"]))
                continue
            if "stop" in msg:
                bcolors.print("STOP", "green")
                comm.wait(["continue"])
            comm.send("whole_data", whole_str)
            if (cam.name == "l515_front" and contains_ring): 
                comm.wait(["continue_ring", "stop"])
            comm.send("fps", time.time() - start)
                
            
        except Exception as e:
            bcolors.print(str(e), "blue")
            comm.open()

finally:
    cam.stop()
