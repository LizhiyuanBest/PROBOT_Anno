#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from probot_demo.srv import Img_process, Img_processResponse
from probot_demo.msg import Process_Result
from probot_demo.msg import Pt2D

pi = 3.14159
ROI = [[200, 400], [650, 900]]  # 框选出的区域，在这个区域内进行识别

def obj_info(color, obj_type, region, cent, angle):
    """ 组装信息 """
    obj = Process_Result()
    obj.color = color
    obj.type = obj_type
    for i in range(len(region)):
        pt = Pt2D()
        pt.x = int(region[i][0]) + ROI[0][1]
        pt.y = int(region[i][1]) + ROI[0][0]
        obj.region[i] = pt
    center = list(cent)
    center[0] += ROI[0][1]
    center[1] += ROI[0][0]
    obj.center = center
    obj.theta = angle
    return obj


def nothing(x):
    pass #在我们的例子中，函数什么都不做，所以我们简单地通过。


def get_blue_color(img):
    """
    get blue color region in hsv image
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # cv2.namedWindow('hsv_image')  # 调参用
    # cv2.createTrackbar('BS','hsv_image',43,255,nothing)
    # cv2.createTrackbar('BV','hsv_image',46,255,nothing)
    # cv2.createTrackbar('BH','hsv_image',124,255,nothing)
    # cv2.createTrackbar('BL','hsv_image',100,255,nothing)
 
    # bv = cv2.getTrackbarPos('BV','hsv_image')
    # bs = cv2.getTrackbarPos('BS','hsv_image')
    # bl = cv2.getTrackbarPos('BL','hsv_image')
    # bh = cv2.getTrackbarPos('BH','hsv_image')
   
    bs,bv = 45,45
    bl,bh = 90,125
    # define range of BGR color in HSV
    threshold_blue = np.array([[bl,bs,bv], [bh,255,255]])
    # Threshold the HSV image to get only blue colors
    mask_blue = cv2.inRange(hsv, threshold_blue[0], threshold_blue[1])
  
    # Bitwise-AND mask and original image
    blue = cv2.bitwise_and(img, img, mask=mask_blue)

    # cv2.imshow('hsv_image', blue)
    # cv2.waitKey(3)

    gray = cv2.cvtColor(blue, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('gray', gray)

    blur = cv2.blur(gray,(3,3)) # 中值滤波
    # cv2.imshow('blur', blur)

    _, thresh = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU) # 二值化
    # cv2.imshow('thresh', thresh)   

    kernel = np.ones((3,3),np.uint8) 
    erosion = cv2.erode(thresh,kernel,iterations=2)  # 腐蚀
    # cv2.imshow('erosion', erosion)

    # cv2.waitKey(3)
    return erosion


class image_process:

    def __init__(self): 
        rospy.init_node('image_process', anonymous=True)
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((720, 1280, 3), dtype=np.uint8)  # 初始图像
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]]
        self.avg_roi = self.img_roi.copy()
        self.cnt_img_roi = self.img_roi.copy() # 结果图像
        self.cnt_img = self.img_src.copy() # 结果图像
        self.objects = Img_processResponse()  # 目标存放位置
        self.objects.result = 'failed'
        self.img_num = 0
        self.time_start = time.time()
        self.imgs = np.zeros((4,ROI[1][0]-ROI[0][0],ROI[1][1]-ROI[0][1],3),dtype=np.uint8) # 存储图像
        
        # 订阅 /camera/color/image_raw 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_sub_callback)
        # 创建一个名为 /image_process/next 的server，注册回调函数 image_srv_callback
        self.image_srv = rospy.Service('/image_process/next', Img_process, self.image_srv_callback)

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]] # ROI
        # print self.img_src.dtype, self.img_src.shape  # 720 1280
        # cv2.imshow("image", self.img_src)
        cv2.imshow("image ROI", self.img_roi)
        self.cnt_img = self.img_src.copy() # 结果图像
        self.cnt_img[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]] = self.cnt_img_roi
        # cv2.imshow("res_image_roi", self.cnt_img_roi)
        cv2.imshow("res_image", self.cnt_img)
        cv2.waitKey(3)

        # 图像处理
        if(self.img_num < 4): # 多副图像去噪
            self.imgs[self.img_num] = self.img_roi
            self.img_num += 1
        else:
            self.img_num = 0
            img = np.mean(self.imgs, dtype=np.int32, axis=0)
            self.avg_roi = np.array(img, dtype=np.uint8)

            blue = get_blue_color(self.avg_roi) # 蓝色区域mask
            obj = self.get_obj(blue) # 物体信息
            self.objects = self.objects_info(obj)  # 封装信息

            # self.time_end = time.time()
            # print('total time: {:.5f}s'.format(self.time_end - self.time_start))  # 0.050
            # self.time_start = time.time() # 开始时间
    
    def filter_contour(self,contours):
        """ 返回轮廓面积大于1000中最大的轮廓 """
        try:
            if (len(contours)>1):
                cnt_area = {}
                max_area = 0
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    # print(area)
                    if area > 1000:
                        cnt_area[area] = cnt
                        if area > max_area:
                            max_area = area
                # print(max_area,cnt_area[max_area])
                return cnt_area[max_area]
            return contours[0]
        except:
            if (len(contours)>1):
                for cnt in contours:
                    area = cv2.contourArea(cnt)
                    if area > 1000:
                        return cnt
            return contours[0]

    def get_obj(self, img):
        """ 得到物体中心和抓取角度 """
        mask = img.astype(np.uint8)
        # 提取轮廓
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # print(len(contours))
        if len(contours) == 0:
            return ['none']
        cnt = self.filter_contour(contours)
        rect = cv2.minAreaRect(cnt) # 得到最小外接矩形的（中心(x,y), (宽,高), 旋转角度）
        box = cv2.boxPoints(rect) # 获取最小外接矩形的4个顶点坐标(ps: cv2.boxPoints(rect) for OpenCV 3.x)
        # print(box)
        center = [np.mean(box[:, 0]), np.mean(box[:, 1])]
        wh = list(rect[1])
        wh.sort()
        ratio = float(wh[0]) / (wh[1] + 1e-10) # 长宽比
        # print("wh", wh, ratio)
        
        # img_show = self.img_roi.copy()  # 显示图像
        box = box.astype(np.int) 
        # 画出来
        # self.cnt_img_roi = cv2.drawContours(self.img_roi, contours, 0, (0, 0, 255), 2)
        self.cnt_img_roi = cv2.drawContours(self.img_roi, [box], 0, (0, 0, 255), 2)

        theta = self.get_theta(box.tolist())
        # print(theta)

        if ratio > 0.65:  # 认为是正方形  # P>0.7 N<0.55
            return ['cube', box.tolist(), center, theta]
        elif ratio <= 0.65: # 长方形
            return ['cuboid', box.tolist(), center, theta]

    def get_theta(self, box):
        # print(box)
        dy1 = box[1][1] - box[0][1]
        dx1 = box[1][0] - box[0][0]
        dy2 = box[2][1] - box[1][1]
        dx2 = box[2][0] - box[1][0]
        k1 = math.atan2(dy1, dx1) / pi * 180
        k2 = math.atan2(dy2, dx2) / pi * 180 
        # print('k1',k1,'k2',k2)
        if (dx1*dx1 + dy1*dy1) > (dx2*dx2 + dy2*dy2):
            k = math.atan2(dy1, dx1) / pi * 180 + 90
        else:
            k = math.atan2(dy2, dx2) / pi * 180 + 90
        return k
    
    def objects_info(self, obj):
        """ 统一格式的目标位置信息 """
        res = Img_processResponse()
        if obj[0] == 'none':  # 没有物体
            res.result = 'faild'
            return res
        # 找到物体
        res.result = 'ok'
        if obj[0] == 'cube':
            res.objects.append(obj_info('blue', 'cube', obj[1], obj[2], obj[3]))
        elif obj[0] == 'cuboid':
            res.objects.append(obj_info('blue', 'cuboid', obj[1], obj[2], obj[3]))

        return res

    def image_srv_callback(self, req):
        # 显示请求数据
        # print('signal', req.signal)
        # 反馈数据
        # blue = get_blue_color(self.avg_roi) # 蓝色区域mask
        # obj = self.get_obj(blue) # 物体信息
        # self.objects = self.objects_info(obj)  # 封装信息

        return self.objects


if __name__ == '__main__':
    img_proc = image_process()
    
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()



