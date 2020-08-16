#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def personInfoCallback(msg):
    cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="passthrough")
    cv2.imshow("image", cv_image)

    cv2.waitKey(3)

    # rospy.loginfo(msg)

def person_subscriber():
	# ROS节点初始化
    rospy.init_node('person_subscriber', anonymous=True)

	# 创建一个Subscriber，订阅名为/person_info的topic，注册回调函数personInfoCallback
    rospy.Subscriber("/probot_anno/camera/image_raw", Image, personInfoCallback)

	# 循环等待回调函数
    rospy.spin()

if __name__ == '__main__':
    person_subscriber()