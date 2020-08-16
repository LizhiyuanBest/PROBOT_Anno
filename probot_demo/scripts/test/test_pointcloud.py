#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray


def point_callback(msg):
    # rospy.loginfo(msg)
    print(msg.height,msg.width)
    print(msg.point_step)
    print(msg.row_step)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('point_subscriber', anonymous=True)

    rospy.Subscriber("/camera/depth/color/points", PointCloud2, point_callback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()