#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import numpy as np
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from nn_vs.msg import Image_Msg
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


ROOT_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images'


class image_listenner:

    def __init__(self): 
        self.image_sub = rospy.Subscriber("/image_data",Image_Msg,self.image_sub_callback)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        self.img = np.ndarray(shape=(data.height, data.width, data.channels), dtype=np.uint8, buffer=data.data)
        cv2.imshow("Image ", self.img)
        c = cv2.waitKey(10)


if __name__ == '__main__':
    image_listenning = image_listenner()
    rospy.init_node('image_listenner', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
    