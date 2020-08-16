#!/usr/bin/env python
# -*- coding: utf-8 -*-


import sys
import rospy
import copy
import math
from std_msgs.msg import Float32MultiArray, String, Float64MultiArray
import cv2
import numpy as np
import time 
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



def camera_callback(data):
    bridge = CvBridge()  # ros和opencv的桥梁
    img_src = bridge.imgmsg_to_cv2(data, "rgb8")
    cv2.imshow("image", img_src)
    cv2.waitKey(3)

# def depth_callback(data):
#     bridge = CvBridge()  # ros和opencv的桥梁
#     img_src = bridge.imgmsg_to_cv2(data, "passthrough")
#     cv2.imshow("depth", img_src)
#     cv2.waitKey(3)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('camera_subscriber', anonymous=True)

    rospy.Subscriber("/kinect2/hd/image_color", Image, camera_callback)

    # rospy.Subscriber("/kinect2/sd/image_depth", Image, depth_callback)
	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()

