import pyrealsense2 as rs
import numpy as np
import cv2

import torch
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator, colors
import time
import colorsys
PATH = "best.pt"
do_depth_ring = False


def return_data(mogos, find="all", colors=[-1,0,1]):
    # Takes in data in the order: [det:[x,y,x,y,dist,color (-1 = red, 0 = yellow, 1 = blue)], det[], .., det[]]
    if find=="all":
        if not colors == [-1,0,1]:
            for i, mogo in enumerate(mogos):
                if not mogo[5] in colors:
                    del mogos[i]
        return mogos
    if find=="close":
        mogos.sort(key=lambda x:x[4])
        for mogo in mogos:
            if mogo[5] in colors:
                return mogo

def convert_rgb_to_hsv(r, g, b):
    color_hsv_percentage=colorsys.rgb_to_hsv(r / float(255), g / float(255), b / float(255)) 
    
    color_h=round(360*color_hsv_percentage[0])
    color_s=round(100*color_hsv_percentage[1])
    color_v=round(100*color_hsv_percentage[2])
    color_hsv=(color_h, color_s, color_v)
    return color_hsv

def determine_color(det):
    bgr = color_image[int(det[1] + (float(det[3] - det[1]) * (2 / 10))):int(det[1] + (float(det[3] - det[1]) * (4.0 / 10))), int(det[0] + (float(det[2] - det[0]) * (4.0 / 10))):int(det[0] + (float(det[2] - det[0]) * (6.0 / 10)))]
    bgr = np.mean(bgr, axis=(0,1))
    hsv = convert_rgb_to_hsv(bgr[2],bgr[1],bgr[0])
    if hsv[0]>=180 and hsv[0]<=240:
        return 1
    elif hsv[0]>=20 and hsv[0]<=100:
        return 0
    elif (hsv[0]>=0 and hsv[0]<20) or (hsv[0]>=320 and hsv[0]<=360):
        return -1
    return 3

def determine_depth(det, do_depth_ring=False):
    depth = []
    for x in range(6):
        for y in range(3):
            depth.append(depth_frame.get_distance(int(det[0] + (float(det[2] - det[0]) * (float(x+3) / 10))),int(det[1] + (float(det[3] - det[1]) * (float(y+2) / 10)))))
    depth = np.array(depth)
    depth = depth[depth>0]
    return np.mean(depth)
        
model = attempt_load(PATH)
device = torch.device("cuda" if torch.cuda.is_available() else 'cpu')
model.to(device)
names = model.module.names if hasattr(model, 'module') else model.names

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


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
        color_image = np.asanyarray(color_frame.get_data())

        # If depth and color resolutions are different, resize color image to match depth image for display
        color_image = cv2.resize(color_image, dsize=(640, 480), interpolation=cv2.INTER_AREA)

        color_image_t = torch.cuda.FloatTensor(color_image)
        color_image_t = torch.moveaxis(color_image_t, 2, 0)[None] / 255.0
        
        pred = model(color_image_t)[0]
        conf_thres = .3
        pred = non_max_suppression(pred, conf_thres)
        pred = pred[0]
        # tensor([[det1],[det2]])
        pred[:,:4] = scale_coords(color_image_t.shape[2:], pred[:, :4], color_image.shape).round()

        for i, det in enumerate(pred):
            if(det[5] == 0): # COLOR
                pred[i, 5] = determine_color(det)
            else:
                pred[i, 5] = 2
            pred[i, 4] = determine_depth(det, do_depth_ring=do_depth_ring)

      
        print(return_data(pred))
        print(time.time()-start)
finally:
    pipeline.stop()
