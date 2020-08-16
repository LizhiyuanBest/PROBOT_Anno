#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32MultiArray
from hsv import get_specific_color


class image_process:

    def __init__(self): 
        rospy.init_node('hand_image_process', anonymous=True)
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((800, 800, 3), dtype=np.uint8)  # 初始图像
        self.cnt_img = self.img_src.copy() # 结果图像
        # 订阅 相机 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/probot_anno/hand_camera/hand_image_raw", Image, self.image_sub_callback)
        self.center_pub = rospy.Publisher('/hand_camera/region',Float32MultiArray)

    def get_center(self, color):
        img = get_specific_color(self.img_src, color) # 检测的颜色 可以改
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = gray.astype(np.uint8)
        # 提取轮廓
        _, contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(contours) == 0:
            return [-10, -10]
        (x,y), (w,h), ang = cv2.minAreaRect(contours[0]) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
         # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
        center = [x, y]
        return center

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # start = time.time() # 开始时间
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        green_center = self.get_center('green')
        blue_center = self.get_center('blue')
        red_center = self.get_center('red')
       
        # 定义发送数据
        region = Float32MultiArray()
        zipped = zip(red_center,green_center,blue_center)  # zip
        for center in zip(*zipped): # unzip
            # print(center)
            region.data.append(center[0])  
            region.data.append(center[1])

        self.center_pub.publish(region)  # 发布消息
        # print(x,y, x+w/2, y+h/2)
        ######################################## 显示图像，占时间可以注释
        self.res_img = self.img_src.copy()  # 显示图像
        cv2.circle(self.res_img, tuple(int(c) for c in red_center), 2, (255, 255, 255), 2)
        cv2.circle(self.res_img, tuple(int(c) for c in  green_center), 2, (255, 255, 255), 2)
        cv2.circle(self.res_img, tuple(int(c) for c in  blue_center), 2, (255, 255, 255), 2)
        cv2.imshow("image", self.img_src) # 显示原图像
        cv2.imshow("res_image", self.res_img) # 显示处理图像
        cv2.waitKey(3)
        ######################################## 显示图像，占时间可以注释
        # end = time.time()
        # print('total time: {:.5f}s'.format(end - start))  # 0.010

           

if __name__ == '__main__':
    img_proc = image_process()
    
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



