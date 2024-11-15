#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
created on 2023-03-15 by lihan
$ pip install roipoly
$ python3 merge_evaluation.py -v 10074C -d 20230307 -p pos1 -l front_lidar -c kb8 -n lihan
$ python3 merge_evaluation.py -v 10074C -d 20230307 -p pos1 -l main_lidar -c kb8 -n lihan

"""


import os
import cv2
import argparse
from PIL import Image
import numpy as np
from matplotlib import pyplot as plt
from roipoly import MultiRoi


def file_exists(filename):
    try:
        with open(filename) as f:
            return True
    except IOError:
        return False
    

def merge_images(imgs, direction="horizontal", gap=0):
    imgs = [Image.fromarray(img) for img in imgs]
    if direction == "horizontal":
        w1, h1 = imgs[0].size
        w2, h2 = imgs[1].size
        w3, h3 = imgs[2].size
        result = Image.new(imgs[0].mode, (w1+w2+w3+2*gap, h1))
        result.paste(imgs[0], box=(0, 0))
        result.paste(imgs[1], box=(w1+gap, 0))
        result.paste(imgs[2], box=(w1+w2+gap, 0))
    elif direction == "vertical":
        w1, h1 = imgs[0].size
        w2, h2 = imgs[1].size
        result = Image.new(imgs[0].mode, (w1, h1+h2+gap))
        result.paste(imgs[0], box=(0, 0))
        result.paste(imgs[1], box=(0, h1+gap))
    else:
        raise ValueError("The direction parameter has only two options: horizontal and vertical")
    return np.array(result)


def select_roi(img, output, prefix):
    plt.imshow(img, interpolation='nearest', cmap="Greys")
    plt.title("click on the button to add a new ROI")
    # draw multiple ROIs
    multiroi_named = MultiRoi(roi_names=[])
    # draw all ROIs
    plt.imshow(img, interpolation='nearest', cmap="Greys")
    roi_names = []
    for name, roi in multiroi_named.rois.items():
        roi.display_roi()
        roi.display_mean(img)
        roi_names.append(name)
    plt.legend(roi_names, bbox_to_anchor=(1.2, 1.05))
    plt.show()
    plt.savefig(output + "/" + prefix + "_roi.jpg") # todo


if __name__ == '__main__':
    # parser argument
    parser = argparse.ArgumentParser(description='merge_evaluation')
    parser.add_argument('-v', '--vehicle', required=True,
                        type=str, help='the information of vehicle')
    parser.add_argument('-d', '--date', required=True,
                        type=str, help='the date of calibration')
    parser.add_argument('-p', '--position', required=True,
                        type=str, help='the position of evaluation')
    parser.add_argument('-l', '--lidar', required=True,
                        type=str, help='the name of lidar')
    parser.add_argument('-c', '--camera', required=True,
                        type=str, help='the model of front-wide-camera (kb8/ocam)')
    parser.add_argument('-n', '--name', required=True,
                        type=str, help='the name of checker')
    args = parser.parse_args()
    vehicle = args.vehicle
    date = args.date
    position = args.position
    lidar = args.lidar
    camera = args.camera
    name = args.name
    
    data_path = "./" + vehicle + "/" + date + "/evaluation/" + position + "/projected/"
    print("data_path:", data_path)
    # save the check result
    output = os.path.join("./" + vehicle + "/" + date + "/evaluation/", name)
    print("save_result:", output)
    if file_exists(output) is False:
        os.makedirs(output, exist_ok=True)
    prefix = vehicle + "_" + date + "_" + lidar + "_" + camera + "_" + position
    print("prefix:", prefix)

    if lidar == "front_lidar":
        print("check the front lidar")
        front_wide = cv2.imread(data_path + camera + "_front_lidar2front_wide.jpg")   # 3840*2160
        front_left = cv2.imread(data_path + "kb8_front_lidar2front_left.jpg")   # 1920*1536
        front_right = cv2.imread(data_path + "kb8_front_lidar2front_right.jpg") # 1920*1536
        # trim images
        left_width = right_width = 1000
        wide_start = 600
        left_start = right_start = 300
        height = 800
        front_wide_trim = front_wide[wide_start:wide_start+height, 0:3840]               
        front_left_trim = front_left[left_start:left_start+height, 1920-left_width:1920] 
        front_right_trim = front_right[right_start:right_start+height, 0:right_width]     
        front_lidar = merge_images([front_left_trim, front_wide_trim, front_right_trim])
        cv2.imwrite(output + "/" + prefix + ".jpg", front_lidar)
        select_roi(front_lidar, output, prefix)
        
    if lidar == "main_lidar":
        print("check the main lidar")
        front_wide = cv2.imread(data_path + camera + "_main_lidar2front_wide.jpg")   # 3840*2160
        front_left = cv2.imread(data_path + "kb8_main_lidar2front_left.jpg")         # 1920*1536
        front_right = cv2.imread(data_path + "kb8_main_lidar2front_right.jpg")       # 1920*1536
        rear = cv2.imread(data_path + "pinhole_main_lidar2rear.jpg")                 # 1920*1536
        rear_left = cv2.imread(data_path + "kb8_main_lidar2rear_left.jpg")           # 1920*1536
        rear_right = cv2.imread(data_path + "kb8_main_lidar2rear_right.jpg")         # 1920*1536
        # trim images
        front_start = 600
        rear_start = left_start = right_start = 300
        height = 800
        front_wide_trim = front_wide[front_start:front_start+height, 0:3840]               
        front_left_trim = front_left[left_start:left_start+height, 0:1920] 
        front_right_trim = front_right[right_start:right_start+height, 0:1920]     
        main_lidar_front = merge_images([front_left_trim, front_wide_trim, front_right_trim], "horizontal")
        rear_trim = rear[rear_start:rear_start+height, 0:1920]               
        rear_left_trim = rear_left[left_start:left_start+height, 0:1920] 
        rear_right_trim = rear_right[right_start:right_start+height, 0:1920]     
        main_lidar_rear = merge_images([rear_right_trim, rear_trim, rear_left_trim], "horizontal")
        main_lidar = merge_images([main_lidar_front, main_lidar_rear], "vertical")
        cv2.imwrite(output + "/" + prefix + ".jpg", main_lidar)
        select_roi(main_lidar, output, prefix)
