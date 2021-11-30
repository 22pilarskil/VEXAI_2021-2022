import colorsys
import numpy as np
import torch


def return_data(mogos, find="all", colors=[-1, 0, 1], close_thresh=200):
    # Takes in data in the order: [det:[x,y,x,y,dist,color (-1 = red, 0 = yellow, 1 = blue)], det[], .., det[]]
    if find == "all":
        if not colors == [-1, 0, 1]:
            for i, mogo in enumerate(mogos):
                if not mogo[5] in colors:
                    del mogos[i]
        return mogos
    if find == "close":
        mogos[mogos != mogos] = 0 #set all nans to 0
        det_0 = mogos[mogos[:,4] == 0] #all zeros
        det_no0 = mogos[mogos[:,4] != 0] #all non-zeros

        if int(det_0.shape[0]) > 0:  #if there are zeros, find max width bounding box
            close_0 = det_0[torch.argmax((det_0[:,2] - det_0[:,0]), dim=0)] 
            print("Det3 Width: ", close_0[2] - close_0[0])
            if close_0[2] - close_0[0] > close_thresh:
                return close_0
            else:
                if(det_no0.shape[0] > 0):
                    return det_no0[torch.argmin(det_no0[:,4], dim=0)]
                else:
                    return close_0            
        else:
            return det_no0[torch.argmin(det_no0[:,4], dim=0)]


        mogos.sort(key=lambda x:x[4])
        for mogo in mogos:
            if mogo[5] in colors:
                return mogo

def convert_rgb_to_hsv(r, g, b):
    color_hsv_percentage = colorsys.rgb_to_hsv(r / float(255), g / float(255), b / float(255)) 
    
    color_h = round(360 * color_hsv_percentage[0])
    color_s = round(100 * color_hsv_percentage[1])
    color_v = round(100 * color_hsv_percentage[2])
    color_hsv = (color_h, color_s, color_v)
    return color_hsv

def determine_color(det, color_image):
    bgr = color_image[int(det[1] + (float(det[3] - det[1]) * (2 / 10))):int(det[1] + (float(det[3] - det[1]) * (4.0 / 10))), int(det[0] + (float(det[2] - det[0]) * (4.0 / 10))):int(det[0] + (float(det[2] - det[0]) * (6.0 / 10)))]
    bgr = np.mean(bgr, axis=(0,1))
    hsv = convert_rgb_to_hsv(bgr[2],bgr[1],bgr[0])
    if hsv[0] >= 180 and hsv[0] <= 240:
        return 1
    elif hsv[0] >= 20 and hsv[0] <= 100:
        return 0
    elif (hsv[0] >= 0 and hsv[0] < 20) or (hsv[0] >= 320 and hsv[0] <= 360):
        return -1
    return 2
    

def determine_depth(det, depth_image, do_depth_ring=False):
    if not do_depth_ring and not det[5] == 2:
        d = depth_image[int(det[1] + (float(det[3] - det[1]) * (2 / 10))):int(det[1] + (float(det[3] - det[1]) * (4.0 / 10))), int(det[0] + (float(det[2] - det[0]) * (4.0 / 10))):int(det[0] + (float(det[2] - det[0]) * (6.0 / 10)))]
        d = d[d > 0]
        return np.mean(d)
    elif do_depth_ring:
        d = depth_image[int(det[1] + (float(det[3] - det[1]) * (2 / 10))):int(det[1] + (float(det[3] - det[1]) * (4.0 / 10))), int(det[0] + (float(det[2] - det[0]) * (4.0 / 10))):int(det[0] + (float(det[2] - det[0]) * (6.0 / 10)))]
        return np.mean(d)
    return -1

def degree(det):
    pixel_degree = 0.109375
    center = 320
    diff = center - (det[2] + det[0]) / 2
    angle = diff * pixel_degree
    return angle

def min(pred):
    return np.argmin(pred[:,4], axis=0)
