import numpy as np
import cv2
import time
import argparse

import torch
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_color, determine_depth, degree, returneth_data
from utils.camera import Camera
from utils.models import Model
from sklearn.cluster import DBSCAN
from utils.decorators import bcolors

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=bool, default=True)
parser.add_argument("--camera", metavar="camera", type=str, default="l515_back")
parser.add_argument("--cluster", metavar="cluster", type=bool, default=False)
args = parser.parse_args()
    
names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]
model = Model("models/weights/best_yolov5n.engine")
conf_thres = .5

cameras = {
    'l515_front': {
        'id': 'f1180887',
        'flip': True,
        },
    'l515_back': {
        'id': 'f1181848',
        'flip': True,
        },
    }

cam = Camera(cameras, args.camera)
cluster = args.cluster

comm = Coms()

try:
    while True:
        start = time.time()

        data, timing = cam.poll_frames()
        if data is None: 
            continue
        color_image, depth_image, color_image_t, depth_colormap, depth_frame = data

        if (time.time() - start) > .20: 
            bcolors.print("Frame Time : {} too long, skipping".format(time.time() - start), "red")
            continue


        pred = model.predict(color_image_t, color_image.shape, conf_thres=conf_thres)
        pred = torch.column_stack((pred, torch.ones(len(pred))))

        for i, det in enumerate(pred):
            if(det[5] == 0): # COLOR
                pred[i, 5] = determine_color(det, color_image)
            else:
                pred[i, 5] = 3
           
            pred[i, 6] = determine_depth(det, depth_image) * depth_frame.get_units()

        data = [0, 0]


        print("Time elapsed: {}".format(time.time() - start))

        color_annotator = Annotator(color_image, line_width=2, pil=not ascii)
        depth_annotator = Annotator(depth_colormap, line_width=2, pil=not ascii)
        xys = []
        det = None

        if int(pred.shape[0]) > 0:
      
            if cluster:
                    
                det = return_data(pred, find="all", colors=[3])
                
                send = ""
                
                for d in det:
                    temp = ""
                    for ids in d:
                        temp += str(round(ids, 2))
                        temp += ","
                    send += "|"
                print(send)
                
        if args.display:
            if det is not None and len(det) > 0:
                color_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(0, True))
                depth_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(0, True))
            color_image, depth_colormap = color_annotator.result(), depth_annotator.result()
            images = np.hstack((color_image, depth_colormap))
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.waitKey(1)

        try:
            switch = comm.read(["camera", "mode"])
            if switch:
                bcolors.print(str(switch), "green")
                if "camera" in switch and not cam.name == switch["camera"]: print(cam.switch_cameras(switch["camera"]))
                if "mode" in switch: cluster = True if switch["mode"] == "true" else False
                continue
            print("Depth: {}, Turn angle: {}".format(data[0], data[1]))
           
            if not data == [0, 0]:
                if cluster:
                    #data.append(frame_time / (time.time() - start))
                    #print(data)
                    comm.send("ring", data)
                    comm.wait('continue')
                elif not cluster:
                    comm.send("mogo", data)
            comm.send("fps", time.time() - start)
        except Exception as e:
            bcolors.print(str(e), "blue")
            comm.open()

          
finally:
    cam.stop()
