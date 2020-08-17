#!/home/li/anaconda3/bin python3
# coding=utf-8

import matplotlib.pyplot as plt
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torchvision
import torch.nn.functional as F
from torch.autograd import Variable
import torch.optim.lr_scheduler as lr_scheduler
from torch.autograd import Variable
import os
from os import listdir, getcwd
from os.path import join
import PIL
import copy
import random
import time
import math
from grasp_net_model import *
from torchvision import transforms as T
import rospy
from nn_vs.srv import Image_data, Image_dataResponse
from sensor_msgs.msg import Image
from nn_vs.msg import Image_Msg
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

print("Pytorch Version: ", torch.__version__)
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

Image_Path = '/home/li/visual_servo/GraspNet/data/images'
Pose_Path = '/home/li/visual_servo/GraspNet/data/labels'
Image_Train_Path = '/home/li/visual_servo/GraspNet/data/images/train'
Train_Path = '/home/li/visual_servo/GraspNet/data/train.txt'
Image_Val_Path = '/home/li/visual_servo/GraspNet/data/images/val'
Val_Path = '/home/li/visual_servo/GraspNet/data/val.txt'
Image_Test_Path = '/home/li/visual_servo/GraspNet/data/images/test'
Test_Path = '/home/li/visual_servo/GraspNet/data/test.txt'
simple_img = '/home/li/visual_servo/GraspNet/data/simple/image_206.jpg'
Weight_Path = '/home/li/visual_servo/GraspNet/weights'
Detect_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/detect'
Blank_file = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/blank.jpg'


model = Net().to(device)
best = 'best.pt'
chkpt = torch.load(join(Weight_Path, best), map_location=device)
model.load_state_dict(chkpt['model'])
del chkpt

model.eval()
blank = cv2.imread(Blank_file)
transform = T.ToTensor()
L = 128
L2 = L // 2
padding = 50

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


def img_pretreated(src):
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
    # lr = L-(rightmost[0]-leftmost[0])
    # tb = L-(bottommost[1]-topmost[1])
    # print(lr,tb)
    # roi_l = leftmost[0]-lr//2 if leftmost[0]-lr//2 > 0 else 0
    # roi_r = rightmost[0]+(lr//2+lr%2) if rightmost[0]+(lr//2+lr%2) < 480 else 480
    # roi_t = topmost[1]-tb//2 if topmost[1]-tb//2 > 0 else 0
    # roi_b = bottommost[1]+(tb//2+tb%2) if bottommost[1]+(tb//2+tb%2) < 640 else 640 
    roi_t = leftmost[0]-50 # if leftmost[0]-50 > 0 else 0
    roi_b = rightmost[0]+50 # if rightmost[0]+50 < 480 else 480
    roi_l = topmost[1]-50 # if topmost[1]-50 > 0 else 0
    roi_r = bottommost[1]+50 # if bottommost[1]+50 < 640 else 640 
    # print(roi_l, roi_r, roi_t, roi_b)
    # ROI (128,128,3)
    return [roi_l,roi_r,roi_t,roi_b]

def img_extend(src_img, blank, L=50):
    width = int(blank.shape[1] + L*2)
    height = int(blank.shape[0] + L*2)
    dim = (width, height)
    ex_blank = cv2.resize(blank, dim)
    ex_blank[L:height-L, L:width-L] = src_img
    return ex_blank


def detectCallback(req):
    print('got data')
    img1 = np.ndarray(shape=(req.img1.height, req.img1.width, req.img1.channels), dtype=np.uint8, buffer=req.img1.data)
    # img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    # img1[:,:,0], img1[:,:,1], img1[:,:,2] = image1[:,:,2],image1[:,:,1],image1[:,:,0]
    # cv2.imshow("Image 1", img1)
    # cv2.waitKey(0)
    img1 = img_extend(img1, blank, L=padding)
    print(img1.shape)
    roi = img_pretreated(img1)
    print(roi)
    img1 = img1[roi[0]:roi[1], roi[2]:roi[3]]
    # cv2.imshow("Img 1", img1)
    # cv2.waitKey(0)
    print(img1.shape)
    # assert img1.shape[0] ==128, 'image.shape must (128,128,3)'
    # assert img1.shape[1] ==128, 'image.shape must (128,128,3)'
    img1 = cv2.cvtColor(img1, cv2.COLOR_BGR2RGB)
    img_src = PIL.Image.fromarray(img1)
    # img2 = transform(img_src).unsqueeze(0)
    # plt.imshow(img_src)
    # plt.show()
    step = 5
    H, W = img_src.size
    # print(H, W)
    imgs = []
    for i in range(L2, W - L2, step):
        STEP = 0
        for j in range(L2, H - L2, step):
            STEP += 1
            img = img_src.crop(box=(j - L2, i - L2, j + L2, i + L2))
            img = transform(img).unsqueeze(0)
            imgs.append(img)
    STEP -= 1
    print(STEP)
    img2 = torch.cat(imgs, 0)
    img2 = img2.to(device)
    pred = model(img2)

    pred_img, pred_ang = pred.split([1, 2], dim=1)
    pred_img = pred_img.squeeze()
    pred_ang = pred_ang.squeeze()

    pred_img = pred_img.cpu().detach().numpy()
    pred_ang = pred_ang.cpu().detach().numpy()
    # print(pred_img.shape, pred_ang.shape)
    pred_image = []
    pred_angle = []
    k = 0
    res_pos_x = []
    res_pos_y = []
    res_ang = []
    for i in range(L2, W-L2, step):
        p_img = []
        p_ang = []
        l = 0
        flag = False
        for j in range(L2, H-L2, step):
            p_pos = pred_img[k*STEP + l]
            p_img.append(p_pos)
            p_ang.append(pred_ang[k*STEP + l])
            l+=1
            if p_pos > 0.99:
                cos2, sin2 = pred_ang[k*STEP + l]
                cos  = math.log(cos2/(1-cos2))
                sin  = 0.5 * math.log((1+sin2)/(1-sin2))
                fai  = 0.5 * math.atan2(sin, cos)
                ang = fai * 180 / math.pi
                # print(p_pos, j, i, ang, end=' ')
                flag = True
                res_pos_x.append(j)
                res_pos_y.append(i)
                res_ang.append(ang)
        # if flag:
        #     print('\n')
        k+=1
        pred_image.append(p_img)
        pred_angle.append(p_ang)

    img_np = np.array(pred_image)
    ang_np = np.array(pred_angle)
    # print('img_np', img_np)
    # print('ang_np', ang_np)
    # print('res_pos_x', res_pos_x)
    # print('res_pos_y', res_pos_y)
    # print('res_ang', res_ang)
    x = np.mean(res_pos_x)
    y = np.mean(res_pos_y)
    ang = np.mean(res_ang)
    x = x + roi[2] - padding
    y = y + roi[0] - padding
    print(x,y,ang)

    # cv2.imshow("img_pos", img_np)
    # cv2.waitKey(3)
    # plt.imshow(img_np)
    # plt.show()
    # plt.imshow(ang_np[:, :, 1])
    # plt.show()
    # plt.imshow(ang_np[:, :, 0])
    # plt.show()

    print('return data')
    res = Image_dataResponse()
    res.pred = [x,y,ang]
	# 反馈数据
    return res


# test()

rospy.init_node('detect_server', anonymous=True)

s = rospy.Service('/detect/server', Image_data, detectCallback)
print('waitting for request.')

# 循环等待回调函数
rospy.spin()
