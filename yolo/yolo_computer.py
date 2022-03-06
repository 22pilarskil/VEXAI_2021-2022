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
PATH = os.getcwd() + "/data/labelbox2yolo/best.pt" #"/data/labelbox2yolo/best.pt"
do_depth_ring = False

import argparse

from utils.models import Model

parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()

model = Model(PATH)

cap = cv2.VideoCapture('/Users/hongm/Downloads/test_video.mp4')
count = 0
try:
    while(cap.isOpened()):
        start = time.time()


        ret, color_frame = cap.read()
        count += 1
        if not count % 10 == 0: continue


        # Convert images to numpy arrays
        color_image = np.asanyarray(color_frame)

        color_image = cv2.resize(color_image, dsize=(640, 640), interpolation=cv2.INTER_AREA)

        color_colormap_dim = color_image.shape
        
#possible necessary change(liam's worked so idk??")
        pred = model.predict(color_image, color_image.shape)

        color_image_t = None
        if torch.cuda.is_available():
            color_image_t = torch.cuda.FloatTensor(color_image)
        else:
            color_image_t = torch.FloatTensor(color_image)
        color_image_t = torch.moveaxis(color_image_t, 2, 0)[None] / 255.0
        
        pred[:,:4] = scale_coords(color_image_t.shape[2:], pred[:, :4], color_image.shape).round()

        start = time.time()
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
                color_annotator.box_label(x[:4], f'{names[int(x[5]) + 1]} {x[4]:.2f}', color=colors(x[5], True))

        if args.display:
            color_image = color_annotator.result()
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)

            print("Time elapsed: {}".format(time.time() - start))
            cv2.waitKey(1)

finally:
    cap.release()
    cv2.destroyAllWindows()
