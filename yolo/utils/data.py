import colorsys
import numpy as np
import torch
from utils.decorators import bcolors
from random import randint
def quicksort(array):

    if len(array) < 2:
        return array

    low, same, high = [], [], []


    pivot = array[randint(0, len(array) - 1)]

    for item in array:

        if item[6] < pivot[6]:
            low.append(item)
        elif item[6] == pivot[6]:
            same.append(item)
        elif item[6] > pivot[6]:
            high.append(item)

    return quicksort(low) + same + quicksort(high)


def sort_distance(list):
    #sort distance (index 6)
    sorted = False
    while(not sorted):
        sorted = True
        for i in range(0, len(list)-1):
            if(list[i][6] > list[i+1][6]):
                sorted = False
                temp = list[i]
                list[i] = list[i+1]
                list[i+1] = temp
    return list
def return_data(mogos, find="all", colors=[-1, 0, 1], conf_thres=1, close_thresh=200):
    # Takes in data in the order: [det:[x,y,x,y,dist,color (-1 = red, 0 = yellow, 1 = blue)], det[], .., det[]]
    mask = []
    #goes through every detection and appends true or false to the mask depending on if it is in colors
    for i, mogo in enumerate(mogos):
        if not mogo[5] in colors: #or mogo[4] < conf_thres:
            mask.append(False)
        else:
            mask.append(True)
        #mask is applied to mogos, only indices that are true in the mask are kept
    mogos = mogos[mask]
    if find == "close":
        mogos = mogos
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
        elif int(det_no0.shape[0]) > 0:
            return det_no0[torch.argmin(det_no0[:,4], dim=0)]
        else:
            return


        mogos.sort(key=lambda x:x[6])
        for mogo in mogos:
            if mogo[5] in colors:
                return mogo
    #If find all return all of the mogos
    return mogos


def convert_rgb_to_hsv(r, g, b):
    color_hsv_percentage = colorsys.rgb_to_hsv(r / float(255), g / float(255), b / float(255))

    color_h = round(360 * color_hsv_percentage[0])
    color_s = round(100 * color_hsv_percentage[1])
    color_v = round(100 * color_hsv_percentage[2])
    color_hsv = (color_h, color_s, color_v)
    return color_hsv



def determine_depth(det, depth_image):
    d = depth_image[int(det[1] + (float(det[3] - det[1]) * (2 / 10))):int(det[1] + (float(det[3] - det[1]) * (4.0 / 10))), int(det[0] + (float(det[2] - det[0]) * (4.0 / 10))):int(det[0] + (float(det[2] - det[0]) * (6.0 / 10)))]
    d = np.array(d[d > 0])
    return np.median(d)

def degree(det):
    pixel_degree = 0.109375
    center = 320
    diff = center - (det[2] + det[0]) / 2
    angle = diff * pixel_degree * 540 / 640
    return angle

def min(pred):
    return np.argmin(pred[:,4], axis=0)
