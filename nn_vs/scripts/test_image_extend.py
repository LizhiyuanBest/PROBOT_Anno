#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import numpy as np
import os
import copy
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')



Image_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images'
ROOT_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images_noshadow_pre'
Blank_file = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/blank.jpg'

if not os.path.exists(ROOT_PATH):
    os.makedirs(ROOT_PATH)

def max_contour(contours):
    """ return the max contour """
    if len(contours) == 0:
        return []
    else:
        max_cnt = []
        max_area = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            # print(area)
            if area > 1000 and area > max_area:
                max_area = area
                max_cnt = cnt
        return max_cnt

def img_pretreated(src, blank):
    img = src.copy()
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # define threshold range of color in HSV
    lower_th = np.array([0,43,0])  
    upper_th = np.array([180,255,255]) 
    # Threshold the HSV image to get only specific colors
    mask = cv2.inRange(hsv, lower_th, upper_th)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)
    # cv2.imshow('hsv_image', res)

    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray', gray)
    blur = cv2.blur(gray,(3,3)) # 中值滤波
    # cv2.imshow('blur', blur)
    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) # 二值化
    # cv2.imshow('thresh', thresh)   
    kernel = np.ones((5,5),np.uint8) 
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel,iterations=1)  # 先腐蚀再膨胀
    # cv2.imshow('erosion', opening)
    dilation = cv2.dilate(opening,kernel,iterations = 1)
    # cv2.imshow('dilation', dilation)

    mask = dilation.astype(np.uint8)
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cont = cv2.drawContours(img, contours, -1, (0,255,0), 3)
    # cv2.imshow('cont', cont)
    cnt = max_contour(contours)
    # 极值点
    leftmost = list(cnt[cnt[:,:,0].argmin()][0])
    rightmost = list(cnt[cnt[:,:,0].argmax()][0])
    topmost = list(cnt[cnt[:,:,1].argmin()][0])
    bottommost = list(cnt[cnt[:,:,1].argmax()][0])
    # print(leftmost, rightmost, topmost, bottommost)
    # ROI
    roi_l = leftmost[0]-50 
    roi_r = rightmost[0]+50 
    roi_t = topmost[1]-50 
    roi_b = bottommost[1]+50 
    print(roi_t,roi_b, roi_l,roi_r)
    return src[roi_t:roi_b, roi_l:roi_r]


def img_extend(src_img, blank, L=50):
    width = int(blank.shape[1] + L*2)
    height = int(blank.shape[0] + L*2)
    dim = (width, height)
    ex_blank = cv2.resize(blank, dim)
    ex_blank[L:height-L, L:width-L] = src_img
    return ex_blank



blank = cv2.imread(Blank_file)

src = cv2.imread(os.path.join(Image_PATH, 'image_201.jpg'))
print(src.shape)
ex_img = img_extend(src,blank,L=50)
print(ex_img.shape)
dst = img_pretreated(ex_img, blank)
cv2.imshow('src', src)
cv2.imshow('ex_img', ex_img)
cv2.imshow('dst', dst)
cv2.waitKey(0)


