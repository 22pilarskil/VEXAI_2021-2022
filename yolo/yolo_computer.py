
import numpy as np
import cv2

import torch
from utils.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator, colors
from utils.data import return_data, determine_color, degree
import time
import os
from sklearn.cluster import DBSCAN

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
cap= cv2.VideoCapture('legg.mp4')

def trt()
{
    color_image = np.asanyarray(color_frame)

    color_image = cv2.resize(color_image, dsize=(640, 640), interpolation=cv2.INTER_AREA)



    color_colormap_dim = color_image.shape


    color_image_t = torch.FloatTensor(color_image)
    color_image_t = torch.moveaxis(color_image_t, 2, 0)[None] / 255.0
    start = time.time()
    pred = model(color_image_t)[0]
    print("TORCH: {}".format(time.time()-start))
    conf_thres = .3
    pred = non_max_suppression(pred, conf_thres)

    color_image0 = color_image
    pred = pred[0]
}

def polling()
{
    color_annotator = Annotator(color_image0, line_width=2, pil=not ascii)
    pred[:,:4] = scale_coords(color_image_t.shape[2:], pred[:, :4], color_image0.shape).round()

    for i, det in enumerate(pred):
        if(det[5] == 0): # COLOR
            pred[i, 5] = determine_color(det, color_image)
        else:
            pred[i, 5] = 3

    names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]

    data = [0, 0]
    xys = []
    cluster = True
    if int(pred.shape[0]) > 0:
        det = return_data(pred, find="all", colors=[-1, 0, 1, 3])

        if len(det) > 0:

            for x in det:
                xys.append([(int(x[2]) + int(x[0])) / 2, (int(x[1])+int(x[3]))/2])

            if cluster:
                clusters = DBSCAN(eps=80, min_samples=2).fit(xys)
                det = np.append(det, clusters.labels_.reshape(clusters.labels_.shape[0],1), axis=1)

            else:
                det = np.append(det, np.zeros((det.shape[0], 1)), axis=1)

            if args.display:
                for i in det:
                    color_annotator.box_label(i[:4], f'{names[int(i[5]) + 1]} {i[4]:.2f} {i[6]}', color=colors(i[5], True))

}

try:
    while(cap.isOpened()):
        start = time.time()


        ret, color_frame = cap.read()


        p = multiprocessing.Process(target=trt)
        p1 = multiprocessing.Process(target=polling)
        p.start()
        p1.start()

        p.join()
        p1.join()

        if args.display:
            color_image = color_annotator.result()
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', color_image)

            print("Time elapsed: {}".format(time.time() - start))
            cv2.waitKey(1)

finally:
    cap.release()
    cv2.destroyAllWindows()
