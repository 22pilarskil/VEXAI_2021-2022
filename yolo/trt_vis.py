import numpy as np
import cv2
import time
import argparse

import torch
from utils.yolo.general import non_max_suppression, scale_coords
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_color, determine_depth, degree
from utils.camera import Camera
from utils.models import Model
from sklearn.cluster import DBSCAN


parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()
    
names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]
model = Model("models/best.engine")


cameras = {
    'l515_front': ('f1180887', True),
    'l515_back': ('f1181848', True),
    }
cam = Camera(cameras, 'l515_front')


comm = Coms()

try:
    while True:
        start = time.time()

        data = cam.poll_frames()
        if data is None: 
            continue
        color_image, depth_image, color_image_t, depth_colormap, depth_frame = data

        pred = model.predict(color_image_t)

        pred = non_max_suppression(pred, conf_thres=.3)[0]
        pred[:,:4] = scale_coords(color_image_t.shape[2:], pred[:, :4], color_image.shape).round()

        for i, det in enumerate(pred):
            if(det[5] == 0): # COLOR
                pred[i, 5] = determine_color(det, color_image)
            else:
                pred[i, 5] = 3
            pred[i, 4] = determine_depth(det, depth_image) * depth_frame.get_units()

        data = [0, 0]


        print("Time elapsed: {}".format(time.time() - start))

        color_annotator = Annotator(color_image, line_width=2, pil=not ascii)
        depth_annotator = Annotator(depth_colormap, line_width=2, pil=not ascii)
        xys = []
        cluster = True
        det = None

        if int(pred.shape[0]) > 0:
            if not cluster:
                det = return_data(pred, find="close", colors=[-1, 0, 1])

                if det is not None and len(det) > 0:
                            
                    if args.display:
                        color_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(det[5], True))
                        depth_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(det[5], True))
                         

                
            if cluster:
                det = return_data(pred, find="all", colors=[3])
                for x in det:
                    xys.append([(int(x[2]) + int(x[0])) / 2, (int(x[1]) + int(x[3])) / 2])
                    color_annotator.box_label(x[:4], f'{names[int(x[5]) + 1]} {x[4]:.2f} {x[6]}', color=colors(x[5], True))
                    depth_annotator.box_label(x[:4], f'{names[int(x[5]) + 1]} {x[4]:.2f} {x[6]}', color=colors(x[5], True))
                add_zeros = True
                if len(xys) > 1:
                    clusters = DBSCAN(eps=80, min_samples=2).fit(xys)
                    cluster_labels = np.array(clusters.labels_)
                    mask1 = cluster_labels != -1
                    cluster_labels = cluster_labels[mask1]
                    if len(cluster_labels) > 0:
                        mask2 = cluster_labels == np.bincount(cluster_labels).argmax()
                        cluster_labels = cluster_labels[mask2]
                        det = det[mask1]
                        det = det[mask2]
                        cluster_pos = np.average(xys, axis=0)
                        det = np.append(det, cluster_labels.reshape(cluster_labels.shape[0],1), axis=1)
                        add_zeros = False

                if add_zeros:
                    det = np.append(det, np.zeros((det.shape[0], 1)), axis=1)

            turn_angle = degree(det)
            if not turn_angle == None:
                data = [float(det[4]), float(turn_angle)]


            if args.display:
                color_image, depth_colormap = color_annotator.result(), depth_annotator.result()
                images = np.hstack((color_image, depth_colormap))
                cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                cv2.imshow('RealSense', images)
                cv2.waitKey(1)


        print("Depth: {}, Turn angle: {}".format(data[0], data[1]))


                
        try:
            if(cluster):
                print("ring")
                comm.send("ring", data)
            else:
                comm.send("mogo", data)
            comm.send("fps", time.time() - start)
            if (comm.read("stop")): 
                while not comm.read("continue"):
                    print("Awaiting \"continue\" signal")
                #switch_cameras(pipeline, config, cameras['l515_front'])
        except:
            comm.open()

           

finally:
    cam.stop()