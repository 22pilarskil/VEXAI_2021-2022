import pyrealsense2 as rs
import numpy as np
import cv2

import torch
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords
from utils.plots import Annotator, colors
import time

PATH = "yolov5s.pt"

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

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

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
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)

        color_image_t = torch.cuda.FloatTensor(color_image)
        color_image_t = torch.moveaxis(color_image_t, 2, 0)[None] / 255.0
        
        pred = model(color_image_t)[0]
        conf_thres = .3
        pred = non_max_suppression(pred, conf_thres)

        color_image0, depth_colormap0 = color_image, depth_colormap

        for i, det in enumerate(pred):

            color_annotator = Annotator(color_image0, line_width=3, pil=not ascii)
            depth_annotator = Annotator(depth_colormap0, line_width=3, pil=not ascii)
            if len(det):
                det[:, :4] = scale_coords(color_image_t.shape[2:], det[:, :4], color_image0.shape).round() #Rescale boxes to original frame size

            for *xyxy, conf, cls in reversed(det):
                c = int(cls)  # integer class
                label = f'{names[c]} {conf:.2f}'
                color_annotator.box_label(xyxy, label, color=colors(c, True))
                depth_annotator.box_label(xyxy, label, color=colors(c, True))

        color_image = color_annotator.result()
        depth_colormap = depth_annotator.result()
        
        images = np.hstack((color_image, depth_colormap))

        cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('RealSense', images)
        print(time.time()-start)
        cv2.waitKey(1)

finally:

    pipeline.stop()
