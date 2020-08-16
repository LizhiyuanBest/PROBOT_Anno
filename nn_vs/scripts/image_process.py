#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import numpy as np
import os
import copy
import cv2


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
    roi_l = leftmost[0]-50 if leftmost[0]-50 > 0 else 0
    roi_r = rightmost[0]+50 if rightmost[0]+50 < 480 else 480
    roi_t = topmost[1]-50 if topmost[1]-50 > 0 else 0
    roi_b = bottommost[1]+50 if bottommost[1]+50 < 640 else 640 
    # 裁剪干扰
    dst = blank.copy()
    dst[roi_t:roi_b, roi_l:roi_r] = src[roi_t:roi_b, roi_l:roi_r]

    return dst

blank = cv2.imread(Blank_file)

imgs_list = os.listdir(Image_PATH)
for img_file in imgs_list:
    # if int(img_file[:-4]) > 936:
    #     continue
    print(img_file)
    src = cv2.imread(os.path.join(Image_PATH, img_file))
    dst = img_pretreated(src, blank)
    cv2.imwrite(os.path.join(ROOT_PATH, img_file), dst)


# src = cv2.imread(os.path.join(Image_PATH, '1.jpg'))
# # print(src.shape)
# dst = img_pretreated(src, blank)
# cv2.imshow('src', src)
# cv2.imshow('dst', dst)
# cv2.waitKey(0)

def nothing(x):
    pass #在我们的例子中，函数什么都不做，所以我们简单地通过。

def test_get_region(img):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.namedWindow('hsv_image')  # 调参用
    cv2.createTrackbar('HL','hsv_image',70,180,nothing)
    cv2.createTrackbar('HH','hsv_image',125,180,nothing)
    cv2.createTrackbar('SL','hsv_image',43,255,nothing)
    cv2.createTrackbar('SH','hsv_image',255,255,nothing)
    cv2.createTrackbar('VL','hsv_image',43,255,nothing)
    cv2.createTrackbar('VH','hsv_image',255,255,nothing)

    hl = cv2.getTrackbarPos('HL','hsv_image')
    hh = cv2.getTrackbarPos('HH','hsv_image')
    sl = cv2.getTrackbarPos('SL','hsv_image')
    sh = cv2.getTrackbarPos('SH','hsv_image')
    vl = cv2.getTrackbarPos('VL','hsv_image')
    vh = cv2.getTrackbarPos('VH','hsv_image')

    # define threshold range of color in HSV
    lower_th = np.array([hl,sl,vl])  
    upper_th = np.array([hh,sh,vh]) 
    # Threshold the HSV image to get only specific colors
    mask = cv2.inRange(hsv, lower_th, upper_th)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow('hsv_image', res)
    cv2.waitKey(3)

