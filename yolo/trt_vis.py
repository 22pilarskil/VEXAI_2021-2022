import numpy as np
import cv2
import time
import argparse

from utils.yolo.general import non_max_suppression, scale_coords
from utils.yolo.plots import Annotator, colors
from utils.serial import Coms
from utils.data import return_data, determine_color, determine_depth, degree
from utils.camera import Camera
from utils.models import Model


parser = argparse.ArgumentParser()
parser.add_argument("--display", metavar="display", type=int, default=1)
args = parser.parse_args()
    
names = ["red-mogo","yellow-mogo", "blue-mogo", "unknown_color", "ring"]
model = Model("models/best.engine")


cameras = {
    'l515_front': ('f1181409', False),
    'l515_back': ('f1181848', True),
    }
cam = Camera(cameras, 'l515_back')


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

        try:
            comm.send("mogo", data)
            comm.send("fps", time.time() - start)
            if (comm.read("stop")): 
                while not comm.read("continue"):
                    print("Awaiting \"continue\" signal")
                #switch_cameras(pipeline, config, cameras['l515_front'])
        except:
            comm.open()

        print("Time elapsed: {}".format(time.time() - start))

        color_annotator = Annotator(color_image, line_width=2, pil=not ascii)
        depth_annotator = Annotator(depth_colormap, line_width=2, pil=not ascii)

        if int(pred.shape[0]) > 0:
            det = return_data(pred, find="close", colors=[-1, 0, 1])

            if det is not None and len(det) > 0:
		                
                if args.display:

                    color_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(det[5], True))
                    depth_annotator.box_label(det[:4], f'{names[int(det[5]) + 1]} {det[4]:.2f}', color=colors(det[5], True))

                    color_image, depth_colormap = color_annotator.result(), depth_annotator.result()
                    images = np.hstack((color_image, depth_colormap))
                    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
                    cv2.imshow('RealSense', images)
                    cv2.waitKey(1)
		             
                turn_angle = degree(det)
                if not turn_angle == None:
                    data = [float(det[4]), float(turn_angle)]
		        
        print("Depth: {}, Turn angle: {}".format(data[0], data[1]))

           

finally:
    cam.stop()
