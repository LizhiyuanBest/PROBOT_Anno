#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from probot_demo.msg import Pt2D
from hsv import get_specific_color

dx = dy = float(400) / 290  # mm/像素  1.37931  可以按照我给你说的哪种方法去设置
pi = 3.141592

u0 = 400
v0 = 400

def get_2D_location(pt):
    """得到相机坐标系下的2D平面坐标, ros 中单位为米"""
    # print pt[0], pt[1]
    # print dx, dy
    x = (pt[0] - u0) * dx
    y = (pt[1] - v0) * dy
    return x/1000, y/1000


class image_process:
    """ 图像处理，得到物体坐标 """
    def __init__(self): 
        rospy.init_node('image_process', anonymous=True)
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((800, 800, 3), dtype=np.uint8)  # 初始图像
        self.cnt_img = self.img_src.copy() # 结果图像
        # 订阅 /probot_anno/camera/image_raw 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/probot_anno/camera/image_raw", Image, self.image_sub_callback)
        # self.center_pub = rospy.Publisher('/hand_camera/center',Pt2D)

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        green = get_specific_color(self.img_src, 'green')
        gray = cv2.cvtColor(green, cv2.COLOR_BGR2GRAY)
        gray = gray.astype(np.uint8)
        _, contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            return
        # self.res_img = cv2.drawContours(green, contours, 0, (125, 255, 255), 2) 
        x,y,w,h = cv2.boundingRect(contours[0])
        self.res_img = self.img_src.copy()
        self.res_img = cv2.rectangle(self.res_img, (x,y), (x+w,y+h), (255,0,0), 2)
        center = [x+w/2, y+h/2] # 图像中的坐标，
        # center = Pt2D()  # 自定义的消息类型，发送中心坐标用
        # center.x = x + w / 2
        # center.y = y + h / 2
        # self.center_pub.publish(center)
        print(center)
        # print(get_2D_location(center)) # 坐标变换
        cv2.imshow("image", self.img_src)
        cv2.imshow("res_image", self.res_img)
        cv2.waitKey(3)


if __name__ == '__main__':
    img_proc = image_process()
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

