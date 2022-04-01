import numpy as np
import cv2
import time
import argparse

import torch
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_color, determine_depth, degree, sort_distance
from utils.camera import Camera
from utils.models import Model
from sklearn.cluster import DBSCAN
from utils.decorators import bcolors

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=bool, default=True)
parser.add_argument("--camera", metavar="camera", type=str, default="l515_back")
parser.add_argument("--cluster", metavar="cluster", type=bool, default=False)



args = parser.parse_args()
whole_str = ""
names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]
model = Model("models/weights/best_yolov5n.engine")
conf_thres = .4

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

        data = cam.poll_frames()
        if data is None:
            continue
        color_image, depth_image, color_image_t, depth_colormap, depth_frame = data


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
            #print("ALKDJLSJLDJKLSDJLKSAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
            det_rings = return_data(pred, find="all", colors=[3])
            if cluster: #apply cluster function
                if det_rings is not None and len(det_rings) > 0:
                    det_rings = det_rings[det_rings[:,5]==3]
                    for x in det_rings:
                        if args.display:
                            color_annotator.box_label(x[:4], f'{names[int(x[5]) + 1]} {x[4]:.2f}', color=colors(x[5], True))
                            depth_annotator.box_label(x[:4], f'{names[int(x[5]) + 1]} {x[4]:.2f}', color=colors(x[5], True))
                        xys.append([(int(x[2]) + int(x[0])) / 2, (int(x[1])+int(x[3]))/2])
                    add_zeros = True
                    if cluster and len(xys)>1:
                        clusters = DBSCAN(eps=80, min_samples=2).fit(xys)
                        cluster_labels = np.array(clusters.labels_)
                        mask1 = cluster_labels!=-1
                        cluster_labels = cluster_labels[mask1]
                        if len(cluster_labels)>0:
                            mask2 = cluster_labels==np.bincount(cluster_labels).argmax()
                            cluster_labels = cluster_labels[mask2]
                            det_rings = det_rings[mask1]
                            det_rings = det_rings[mask2]
                            cluster_pos = np.average(xys, axis=0)
                            det_rings = np.append(det_rings, cluster_labels.reshape(cluster_labels.shape[0],1), axis=1)
                            add_zeros = False


                    if add_zeros:
                        det_rings = np.append(det_rings, np.zeros((det_rings.shape[0], 1)), axis=1)
                    det_rings = torch.tensor(det_rings)
                    det_rings = return_data(det_rings, find="all", colors = [3])
                    det_rings = sort_distance(det_rings)
            det_red = return_data(pred, find="all", colors=[-1], conf_thres=conf_thres)
            det_red = sort_distance(det_red)
            det_blue = return_data(pred, find="all",colors = [1], conf_thres = conf_thres)
            det_blue = sort_distance(det_blue)
            det_yellow = return_data(pred, find = "all",colors =[0],conf_thres = conf_thres)
            det_yellow = sort_distance(det_yellow)

            #if pred is not None and len(pred) > 0:
            #load strings into the data in form rings, reds, yellows, blues
            for i, single_det in enumerate(det_rings):
                turn_angle = degree(single_det)
                det_str = str(round(float(single_det[6]), 3)) +  "," + str(round(float(turn_angle), 3)) + "|"
                whole_str = whole_str+det_str

            whole_str = whole_str+ "!"
            for i, single_det in enumerate(det_red):
                turn_angle = degree(single_det)
                det_str = str(round(float(single_det[6]), 3)) +  "," + str(round(float(turn_angle), 3)) + "|"
                whole_str = whole_str+det_str

            whole_str = whole_str+"!"
            for i, single_det in enumerate(det_yellow):
                turn_angle = degree(single_det)
                det_str = str(round(float(single_det[6]), 3)) +  "," + str(round(float(turn_angle), 3)) + "|"
                whole_str = whole_str+det_str

            whole_str = whole_str+"!"
            for i, single_det in enumerate(det_blue):
                turn_angle = degree(single_det)
                det_str = str(round(float(single_det[6]), 3)) +  "," + str(round(float(turn_angle), 3)) + "|"
                whole_str = whole_str+det_str

            whole_str = whole_str+"!"



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
            comm.send("fps", time.time() - start)

            comm.send("whole_data", whole_str)
        except Exception as e:
            bcolors.print(str(e), "blue")
            comm.open()

#sort by distance within color, sort by color/ring
finally:
    cam.stop()
