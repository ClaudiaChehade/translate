#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
created on 2023-04-10 by lihan
$ pip install opencv-python
$ pip install Pillow
"""

import os
import cv2
import copy
import argparse
import numpy as np
from PIL import Image

Image.MAX_IMAGE_PIXELS = None


class Rect(object):
    def __init__(self):
        self.tl = (0, 0)
        self.br = (0, 0)

    def regularize(self):
        pt1 = (min(self.tl[0], self.br[0]), min(self.tl[1], self.br[1]))
        pt2 = (max(self.tl[0], self.br[0]), max(self.tl[1], self.br[1]))
        self.tl = pt1
        self.br = pt2


class DrawRects(object):
    def __init__(self, image, color, thickness=1):
        self.original_image = image
        self.image_for_show = image.copy()
        self.color = color
        self.thickness = thickness
        self.rects = []
        self.current_rect = Rect()
        self.left_button_down = False

    @staticmethod
    def __clip(value, low, high):
        output = max(value, low)
        output = min(output, high)
        return output

    def shrink_point(self, x, y):
        height, width = self.image_for_show.shape[0:2]
        x_shrink = self.__clip(x, 0, width)
        y_shrink = self.__clip(y, 0, height)
        return (x_shrink, y_shrink)

    def append(self):
        self.rects.append(copy.deepcopy(self.current_rect))

    def pop(self):
        rect = Rect()
        if self.rects:
            rect = self.rects.pop()
        return rect

    def reset_image(self):
        self.image_for_show = self.original_image.copy()

    def draw(self):
        for rect in self.rects:
            cv2.rectangle(self.image_for_show, rect.tl, rect.br, color=self.color, thickness=self.thickness)

    def draw_current_rect(self):
        cv2.rectangle(self.image_for_show, self.current_rect.tl, self.current_rect.br, color=self.color, thickness=self.thickness)


def onmouse_draw_rect(event, x, y, flags, draw_rects):
    if event == cv2.EVENT_LBUTTONDOWN:
        draw_rects.left_button_down = True
        draw_rects.current_rect.tl = (x, y)
    if draw_rects.left_button_down and event == cv2.EVENT_MOUSEMOVE:
        draw_rects.current_rect.br = draw_rects.shrink_point(x, y)
        draw_rects.reset_image()
        draw_rects.draw()
        draw_rects.draw_current_rect()
    if event == cv2.EVENT_LBUTTONUP:
        draw_rects.left_button_down = False
        draw_rects.current_rect.br = draw_rects.shrink_point(x, y)
        draw_rects.current_rect.regularize()
        draw_rects.append()
    if (not draw_rects.left_button_down) and event == cv2.EVENT_RBUTTONDOWN:
        draw_rects.pop()
        draw_rects.reset_image()
        draw_rects.draw()


def merge_two_image(image1, image2, direction="horizontal", gap=0):
    img1 = Image.fromarray(image1) 
    img2 = Image.fromarray(image2) 
    w1, h1 = img1.size
    w2, h2 = img2.size
    if direction == "horizontal":    
        result = Image.new(img1.mode, (w1+w2+gap, h1))
        result.paste(img1, box=(0, 0))
        result.paste(img2, box=(w1+gap, 0))
    elif direction == "vertical":
        result = Image.new(img1.mode, (w1, h1+h2+gap))
        result.paste(img1, box=(0, 0))
        result.paste(img2, box=(0, h1+gap))
    else:
        raise ValueError("The direction parameter has only two options: horizontal and vertical")
    return np.array(result)


def merge_images(images, direction="horizontal"):
    merge = merge_two_image(images[0], images[1], direction)
    for i in range(len(images) - 2):
        merge = merge_two_image(merge, images[i + 2], direction)
    return merge


def get_images(path, projects, suffix):
    images = []
    for i in projects:
        data = data_path + i + suffix
        if os.path.exists(data) is True:
            images.append(cv2.imread(data))
        else:
            print("can not find: ", data) 
            image_width = 0
            image_height = 0
            if (i.find("front_wide") == -1): 
                image_width = 1920
                image_height = 1536
            else: 
                image_width = 3840
                image_height = 2160                     
            images.append(np.zeros((image_height, image_width, 3), np.uint8))
    return images
         

if __name__ == '__main__':
    # parser argument
    parser = argparse.ArgumentParser(description='check_front_tele_camera')
    parser.add_argument('-v', '--vehicle', required=True, type=str, help='the information of vehicle')
    parser.add_argument('-d', '--date', required=True, type=str, help='the date of calibration')
    parser.add_argument('-p', '--position', required=True, type=str, help='the positions of evaluation')
    parser.add_argument('-a', '--asample', required=True, type=str, help='a-sample yes or no')
    parser.add_argument('-m', '--mark', required=True, type=str, help='mark yes or no')
    args = parser.parse_args()
    vehicle = args.vehicle
    date = args.date
    positions = args.position.split('/')
    print("the number of positions:", len(positions))
    is_asample = args.asample
    is_mark = args.mark
    prefix = vehicle + "_" + date + "_"
    
    projects = []
    if is_asample == "y":
        projects = ['kb8_front_lidar2front_left',
                    'kb8_front_lidar2front_right',
                    'kb8_front_lidar2front_wide',
                    'kb8_left_lidar2front_left',
                    'kb8_left_lidar2front_wide',
                    'kb8_right_lidar2front_right',
                    'kb8_right_lidar2front_wide',
                    'ocam_front_lidar2nrcs_front',
                    'ocam_left_lidar2nrcs_front',
                    'ocam_left_lidar2nrcs_left',
                    'ocam_right_lidar2nrcs_front',
                    'ocam_right_lidar2nrcs_right',
                    'pinhole_front_lidar2front_tele',
                    'pinhole_left_lidar2front_tele',
                    'pinhole_right_lidar2front_tele']
    else:
        projects = ['kb8_front_lidar2front_left',
                    'kb8_front_lidar2front_right',
                    'kb8_front_lidar2front_wide',
                    'kb8_front_lidar2roof_front_left',
                    'kb8_front_lidar2roof_front_right',
                    'kb8_left_lidar2front_left',
                    'kb8_left_lidar2front_wide',
                    'kb8_left_lidar2roof_front_left',
                    'kb8_main_lidar2front_left',
                    'kb8_main_lidar2front_right',
                    'kb8_main_lidar2front_wide',
                    'kb8_main_lidar2rear_left',
                    'kb8_main_lidar2rear_right',
                    'kb8_main_lidar2roof_front_left',
                    'kb8_main_lidar2roof_front_right',
                    'kb8_qt128_front_lidar2front_left',
                    'kb8_qt128_front_lidar2front_right',
                    'kb8_qt128_front_lidar2front_wide',
                    'kb8_qt128_front_lidar2roof_front_left',
                    'kb8_qt128_front_lidar2roof_front_right',
                    'kb8_qt128_left_lidar2front_left',
                    'kb8_qt128_left_lidar2front_wide',
                    'kb8_qt128_left_lidar2rear_left',
                    'kb8_qt128_left_lidar2roof_front_left',
                    'kb8_qt128_rear_lidar2rear_left',
                    'kb8_qt128_rear_lidar2rear_right',
                    'kb8_qt128_right_lidar2front_right',
                    'kb8_qt128_right_lidar2front_wide',
                    'kb8_qt128_right_lidar2rear_right',
                    'kb8_qt128_right_lidar2roof_front_right',
                    'kb8_right_lidar2front_right',
                    'kb8_right_lidar2front_wide',
                    'kb8_right_lidar2roof_front_right',
                    'ocam_front_lidar2nrcs_front',
                    'ocam_left_lidar2nrcs_front',
                    'ocam_left_lidar2nrcs_left',
                    'ocam_main_lidar2nrcs_front',
                    'ocam_main_lidar2nrcs_left',
                    'ocam_main_lidar2nrcs_rear',
                    'ocam_main_lidar2nrcs_right',
                    'ocam_qt128_front_lidar2nrcs_front',
                    'ocam_qt128_left_lidar2nrcs_left',
                    'ocam_qt128_rear_lidar2nrcs_rear',
                    'ocam_qt128_right_lidar2nrcs_right',
                    'ocam_right_lidar2nrcs_front',
                    'ocam_right_lidar2nrcs_right',
                    'pinhole_front_lidar2front_tele',
                    'pinhole_left_lidar2front_tele',
                    'pinhole_main_lidar2add_rear',
                    'pinhole_main_lidar2front_tele',
                    'pinhole_main_lidar2rear',
                    'pinhole_qt128_left_lidar2add_rear',
                    'pinhole_qt128_left_lidar2rear',
                    'pinhole_qt128_rear_lidar2add_rear',
                    'pinhole_qt128_rear_lidar2rear',
                    'pinhole_qt128_right_lidar2add_rear',
                    'pinhole_qt128_right_lidar2rear',
                    'pinhole_right_lidar2front_tele']

    if is_mark == "y":
        for p in positions:
            data_path = "./" + vehicle + "/" + date + "/evaluation/" + p + "/projected/"
            for i in projects:
                data = data_path + i + '.jpg'
                if os.path.exists(data) is True:
                    image = cv2.imread(data)
                    draw_rects = DrawRects(image, (255, 255, 255), 5)
                    win_name = 'mark check result'
                    cv2.namedWindow(win_name, 0)
                    cv2.resizeWindow(win_name, 1500, 1000) 
                    cv2.setMouseCallback(win_name, onmouse_draw_rect, draw_rects)
                    while True:
                        img = draw_rects.image_for_show
                        cv2.putText(img, p + " " + i, (100, int(img.shape[0]/2)), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 5)                   
                        cv2.imshow(win_name, img)
                        key = cv2.waitKey(60)
                        if key == 27:
                            cv2.imwrite(data_path + i + '_mark.jpg', img)
                            break
                    cv2.destroyAllWindows()                    
                else:
                    print("can not find: ", data) 
                    continue                
                
    suffix = ".jpg"
    if is_mark == "y":
        suffix = "_mark.jpg"
    
    print("check the front tele camera")    
    if is_asample == "y":
        projects = ['pinhole_left_lidar2front_tele',
                    'pinhole_front_lidar2front_tele',
                    'pinhole_right_lidar2front_tele']
    else:
        projects = ['pinhole_main_lidar2front_tele',
                    'pinhole_left_lidar2front_tele',
                    'pinhole_front_lidar2front_tele',
                    'pinhole_right_lidar2front_tele']

    all_position = []
    for p in positions:
        data_path = "./" + vehicle + "/" + date + "/evaluation/" + p + "/projected/"     
        output = "./" + vehicle + "/" + date + "/evaluation/"
        images = get_images(data_path, projects, suffix) # 3840*2160
        all_position.append(merge_images(images))
    front_tele_camera = merge_images(all_position, "vertical")
    resize = cv2.resize(front_tele_camera, (int(front_tele_camera.shape[1]*0.5), int(front_tele_camera.shape[0]*0.5)))
    cv2.imwrite(output + "/" + prefix + "front_tele_camera.jpg", resize)

    
    height = 800
    wide_start = 600
    front_start = 600
    rear_start = left_start = right_start = 300
    left_width = right_width = 1100
    
    if is_asample == "n":
        print("check the main lidar")
        projects = ['kb8_main_lidar2front_wide',
                    'kb8_main_lidar2front_left',
                    'kb8_main_lidar2front_right',
                    'pinhole_main_lidar2rear',
                    'kb8_main_lidar2rear_left',
                    'kb8_main_lidar2rear_right']
        all_position.clear()
        for p in positions:
            data_path = "./" + vehicle + "/" + date + "/evaluation/" + p + "/projected/"            
            images = get_images(data_path, projects, suffix)
            front_wide_trim = images[0][front_start:front_start+height, 0:3840]  # 1920*1536               
            front_left_trim = images[1][left_start:left_start+height, 0:1920]    # 3840*2160
            front_right_trim = images[2][right_start:right_start+height, 0:1920] # 1920*1536    
            main_lidar_front = merge_images([front_left_trim, front_wide_trim, front_right_trim], "horizontal") 
            rear_trim = images[3][rear_start:rear_start+height, 0:1920]         # 1920*1536              
            rear_left_trim = images[4][left_start:left_start+height, 0:1920]    # 1920*1536
            rear_right_trim = images[5][right_start:right_start+height, 0:1920] # 1920*1536    
            main_lidar_rear = merge_images([rear_right_trim, rear_trim, rear_left_trim], "horizontal")
            one_position = merge_images([main_lidar_front, main_lidar_rear], "vertical")
            all_position.append(one_position)
        main_lidar = merge_images(all_position, "vertical")    
        cv2.imwrite(output + "/" + prefix + "main_lidar.jpg", main_lidar)
    
    
    print("check the front/left/right lidar")
    projects_fl = ['kb8_front_lidar2front_wide',
                   'kb8_front_lidar2front_left',
                   'kb8_front_lidar2front_right']
    projects_ll = ['kb8_left_lidar2front_wide',
                   'kb8_left_lidar2front_left']                
    projects_rl = ['kb8_right_lidar2front_wide',
                   'kb8_right_lidar2front_right']                                                   
    front_all_position = []
    left_all_position = []
    right_all_position = []
    for p in positions:
        data_path = "./" + vehicle + "/" + date + "/evaluation/" + p + "/projected/"
        # front_lidar
        images = get_images(data_path, projects_fl, suffix)
        front_wide_trim = images[0][wide_start:wide_start+height, 0:3840]               # 3840*2160          
        front_left_trim = images[1][left_start:left_start+height, 1920-left_width:1920] # 1920*1536
        front_right_trim = images[2][right_start:right_start+height, 0:right_width]     # 1920*1536 
        one_position_1 = merge_images([front_left_trim, front_wide_trim, front_right_trim])
        front_all_position.append(one_position_1)        
        
        # left_lidar
        images = get_images(data_path, projects_ll, suffix)        
        left_wide_trim = images[0][wide_start:wide_start+height, 0:3840]               # 3840*2160             
        left_left_trim = images[1][left_start:left_start+height, 1920-left_width:1920] # 1920*1536     
        one_position = merge_images([left_left_trim, left_wide_trim])
        left_all_position.append(one_position)    
        
        # right_lidar
        images = get_images(data_path, projects_rl, suffix)            
        right_wide_trim = images[0][wide_start:wide_start+height, 0:3840]           # 3840*2160              
        right_right_trim = images[1][right_start:right_start+height, 0:right_width] # 1920*1536    
        one_position = merge_images([right_wide_trim, right_right_trim])
        right_all_position.append(one_position)
    
    front_lidar = merge_images(front_all_position, "vertical") 
    left_lidar = merge_images(left_all_position, "vertical") 
    right_lidar = merge_images(right_all_position, "vertical")    
    cv2.imwrite(output + "/" + prefix + "front_lidar.jpg", front_lidar)
    cv2.imwrite(output + "/" + prefix + "left_lidar.jpg", left_lidar) 
    cv2.imwrite(output + "/" + prefix + "right_lidar.jpg", right_lidar)
