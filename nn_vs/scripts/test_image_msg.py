#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import numpy as np
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nn_vs.msg import Image_Msg 
import cv2

class image_listenner:

    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/probot_anno/hand_camera/hand_image_raw",Image,self.image_sub_callback)
        self.image_pub = rospy.Publisher('/image_data', Image_Msg, queue_size=1)
        self.img = np.zeros((480, 640, 3), dtype=np.uint8)  # 初始图像

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            size = self.img.shape
            image = Image_Msg()
            image.height = size[0] # 480
            image.width = size[1] # 640
            image.channels = size[2] # 3
            image.data = self.bridge.cv2_to_imgmsg(self.img, encoding="bgr8").data
            self.image_pub.publish(image)
            cv2.imshow("Image window", self.img)
            cv2.waitKey(10)

        except CvBridgeError as e:
            print(e) 



if __name__ == '__main__':
    rospy.init_node('image_listenner', anonymous=True)
    image_listenning = image_listenner()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
    