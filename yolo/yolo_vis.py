import pyrealsense2 as rs
import numpy as np
import cv2

import torch
from models.experimental import attempt_load
from utils.yolo.general import non_max_suppression, scale_coords
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_color, determine_depth, degree
from utils.camera import initialize_config, switch_cameras
import time
import os

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
        

        color_image_t = torch.cuda.FloatTensor(color_image)
        color_image_t = torch.moveaxis(color_image_t, 2, 0)[None] / 255.0
        start = time.time()
        pred = model(color_image_t)[0]
        print("TORCH: {}".format(time.time()-start))
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

            print("Units {}".format(depth_frame.get_units()))

        names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]

        data = [0, 0]
        xys = []
        cluster = True
        if int(pred.shape[0]) > 0:
            det = return_data(pred, find="all", colors=[-1, 0, 1, 3])

            if len(det) > 0:
                if cluster:
                    det = det[det[:,5]==3]
                for x in det:
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
                        det = det[mask1]
                        det = det[mask2]
                        cluster_pos = np.average(xys, axis=0)
                        det = np.append(det, cluster_labels.reshape(cluster_labels.shape[0],1), axis=1)
                        add_zeros = False


                if add_zeros:
                    det = np.append(det, np.zeros((det.shape[0], 1)), axis=1)

                if args.display:
                    for i in det:
                        color_annotator.box_label(i[:4], f'{names[int(i[5]) + 1]} {i[4]:.2f} {i[6]}', color=colors(i[5], True))
                det = return_data(det, find="close", colors=[-1, 0, 1, 3])
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

