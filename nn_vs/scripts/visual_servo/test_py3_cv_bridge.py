#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import os
from os import listdir, getcwd
from os.path import join

import copy
import random
import math

import rospy
from nn_vs.srv import Detect_data, Detect_dataResponse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


Detect_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/detect'


# ROS 
bridge = CvBridge()

def detectCallback(req):
    print('got data')
	# 显示请求数据
    # python 调用cv_bridge 有一些问题
    img1 = bridge.imgmsg_to_cv2(req.img1, "bgr8")
    img2 = bridge.imgmsg_to_cv2(req.img2, "bgr8")
    print('got cv img')
    img1 = cv2.imread(join(Detect_PATH, 'img1.jpg'))
    img2 = cv2.imread(join(Detect_PATH, 'img2.jpg'))
    cv2.imshow("img1", img1)
    cv2.imshow("img2", img2)
    cv2.waitKey(0)

   
    print('return data')
    res = Detect_dataResponse()
    res.pred = 0.0
	# 反馈数据
    return res


rospy.init_node('detect_server', anonymous=True)

s = rospy.Service('/detect/server', Detect_data, detectCallback)
print('waitting for request.')
# 循环等待回调函数
rospy.spin()


