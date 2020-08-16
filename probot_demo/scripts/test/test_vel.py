#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray, Float64MultiArray




def person_subscriber():
	# ROS节点初始化
    rospy.init_node('hand_camera_subscriber', anonymous=True)

    vel_pub = rospy.Publisher("/probot_anno/arm_vel_controller/command", Float64MultiArray, queue_size=10)
 

    pub_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # pub_msg.data.append(0.1)

    vel_pub.publish(pub_msg)


if __name__ == '__main__':
    while True:
        person_subscriber()