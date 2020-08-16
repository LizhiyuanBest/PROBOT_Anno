#!/home/li/anaconda3/bin python3
# coding=utf-8

import matplotlib.pyplot as plt
import numpy as np
import torch
from torchvision import transforms as T
import torch.optim as optim
import torchvision
from parse_pose import *
from model import Net
import os
from os import listdir, getcwd
from os.path import join
import PIL 
import copy
import random
import math
from utils import *

import rospy
from nn_vs.srv import Detect_data, Detect_dataResponse
from nn_vs.msg import Image_Msg
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


print("Pytorch Version: ", torch.__version__)

Weights_PATH = '/home/li/visual_servo/delta_pose/weights'
weights_file = 'models_epoch_1_1440000.pt'
Detect_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/detect'

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

alexnet_model = torchvision.models.alexnet(pretrained=False)
model = Net(alexnet_model).to(device)

if weights_file in os.listdir(Weights_PATH):
    model.load_state_dict(torch.load(join(Weights_PATH, weights_file), map_location=device))
    print('load weights')
model.eval()


transform = T.ToTensor()

def detectCallback(req):
    print('got data')
    image1 = np.ndarray(shape=(req.img1.height, req.img1.width, req.img1.channels), dtype=np.uint8, buffer=req.img1.data)
    image2 = np.ndarray(shape=(req.img2.height, req.img2.width, req.img2.channels), dtype=np.uint8, buffer=req.img2.data)
    
    img1 = np.zeros((480, 640, 3), dtype=np.uint8)
    img2 = np.zeros((480, 640, 3), dtype=np.uint8)
    img1[:,:,0], img1[:,:,1], img1[:,:,2] = image1[:,:,2],image1[:,:,1],image1[:,:,0]
    img2[:,:,0], img2[:,:,1], img2[:,:,2] = image2[:,:,2],image2[:,:,1],image2[:,:,0]
    # cv2.imshow("Image 1", img1)
    # cv2.imshow("Image 2", img2)
    # cv2.waitKey(0)

    img1 = PIL.Image.fromarray(img1)
    img2 = PIL.Image.fromarray(img2)
    img1 = transform(img1).unsqueeze(0)
    img2 = transform(img2).unsqueeze(0)
    img1 = img1.to(device)
    img2 = img2.to(device)
    # pred
    pred = model(img1, img2)
    # print(pred.shape)
       
    pred = pred.cpu().detach().numpy()
    pred = pred.tolist()
    print('pred',pred[0]) 
    
    print('return data')
    res = Detect_dataResponse()
    res.pred = pred[0]
	# 反馈数据
    return res


rospy.init_node('detect_server', anonymous=True)

s = rospy.Service('/detect/server', Detect_data, detectCallback)
print('waitting for request.')
# 循环等待回调函数
rospy.spin()




