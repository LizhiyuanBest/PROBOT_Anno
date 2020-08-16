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

model = Net().to(device)
best = 'best.pt'
chkpt = torch.load(join(Weight_Path, best), map_location=device)
model.load_state_dict(chkpt['model'])
del chkpt

model.eval()

transform = T.ToTensor()
L = 128
L2 = L // 2

def detectCallback(req):
    print('got data')
    image1 = np.ndarray(shape=(req.img1.height, req.img1.width, req.img1.channels), dtype=np.uint8, buffer=req.img1.data)
    img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    img1[:,:,0], img1[:,:,1], img1[:,:,2] = image1[:,:,2],image1[:,:,1],image1[:,:,0]
    # cv2.imshow("Image 1", img1)
    # cv2.waitKey(0)

    img_src = PIL.Image.fromarray(img1)
    # img1 = transform(img1).unsqueeze(0)
    # plt.imshow(img_src)
    # plt.show()

    H, W = img_src.size
    print(H, W)
    imgs = []
    for i in range(L2, W - L2, 10):
        for j in range(L2, H - L2, 10):
            img = img_src.crop(box=(j - L2, i - L2, j + L2, i + L2))
            img = transform(img).unsqueeze(0)
            imgs.append(img)

    img2 = torch.cat(imgs, 0)
    img2 = img2.to(device)
    pred = model(img2)

    pred_img, pred_ang = pred.split([1, 2], dim=1)
    pred_img = pred_img.squeeze()
    pred_ang = pred_ang.squeeze()

    pred_img = pred_img.cpu().detach().numpy()
    pred_ang = pred_ang.cpu().detach().numpy()

    pred_image = []
    pred_angle = []
    k = 0
    res_pos_x = []
    res_pos_y = []
    res_ang = []
    for i in range(L2, W-L2, 10):
        p_img = []
        p_ang = []
        l = 0
        flag = False
        for j in range(L2, H-L2, 10):
            p_pos = pred_img[k*((H-L)//10+1) + l]
            p_img.append(p_pos)
            p_ang.append(pred_ang[k*((H-L)//10+1) + l])
            l+=1
            if p_pos > 0.95:
                cos2, sin2 = pred_ang[k*((H-L)//10+1) + l]
                cos  = math.log(cos2/(1-cos2))
                sin  = 0.5 * math.log((1+sin2)/(1-sin2))
                fai  = 0.5 * math.atan2(sin, cos)
                ang = fai * 180 / math.pi
                # print(j,i, ang, end=' ')
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

    x = np.mean(res_pos_x)
    y = np.mean(res_pos_y)
    ang = np.mean(res_ang)
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
