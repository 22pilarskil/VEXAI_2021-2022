import numpy as np
import cv2
import torch
from models.experimental import attempt_load
from utils.yolo.general import non_max_suppression, scale_coords
from utils.yolo.plots import Annotator, colors
from utils.data import return_data, determine_color, determine_depth, degree
import time
import os
from sklearn.cluster import DBSCAN
PATH = os.getcwd() + "/models/weights/best_yolov5n.pt"
do_depth_ring = False

import argparse

from utils.models import Model

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()

model = Model(PATH)

cap = cv2.VideoCapture('/Users/michaelpilarski/Downloads/20220214_150616.mp4')
count = 0
try:
    while(cap.isOpened()):
        start = time.time()


        ret, color_frame = cap.read()
        count += 1
        if not count % 5 == 0: continue


        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame)

        color_image = cv2.resize(color_image, dsize=(640, 640), interpolation=cv2.INTER_AREA)

        color_colormap_dim = color_image.shape
        
        pred = model.predict(color_image, color_image.shape)


        print("TORCH: {}".format(time.time()-start))

        color_image0 = color_image

        color_annotator = Annotator(color_image0, line_width=2, pil=not ascii)
        print("TORCH: {}".format(time.time()-start))

        color_image0 = color_image

        color_annotator = Annotator(color_image0, line_width=2, pil=not ascii)

        for i, det in enumerate(pred):
            if(det[5] == 0): # COLOR
                pred[i, 5] = determine_color(det, color_image)
            else:
                pred[i, 5] = 3

        names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]

        if int(pred.shape[0]) > 0:
            for x in pred:
                index = 0 if names[int(x[5]) + 1] == "ring" else 5
                color_annotator.box_label(x[:4], f'{names[int(x[5]) + 1]} {x[4]:.2f}', color=colors(index, True))

        if args.display:
            color_image = color_annotator.result()
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)

            print("Time elapsed: {}".format(time.time() - start))
            cv2.waitKey(1)

finally:
    cap.release()
    cv2.destroyAllWindows()
