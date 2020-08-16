#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from real_find_object import * 
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from probot_demo.srv import Img_process, Img_processResponse
from probot_demo.msg import Process_Result
from probot_demo.msg import Pt2D
from hsv import get_specific_color


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


def cir_region(center, r):
    """ 确定圆所在的区域，近似为正方形 """
    xc, yc = center
    region = [[xc-r, yc+r], [xc-r, yc-r], [xc+r, yc-r], [xc+r, yc+r]]
    return region


def noknow_info(color, cnt):
    """ 得到未知轮廓的位置信息 """
    rect = cv2.minAreaRect(cnt)
    box = cv2.boxPoints(rect)
    box = box.astype(np.int)
    center = get_center([cnt])
    return obj_info(color, 'noknow', box, center[0], rect[-1])


def objects_info(contours):
    """ 将轮廓信息转化为统一格式的目标位置信息 """
    res = Img_processResponse()
    res.result = 'ok'
    for color, cnts in contours.items():
        # print('cnts', cnts)
        for cnt in cnts:
            # print('cnt', cnt)
            if cnt[0] == 'cylinder':
                res.objects.append(obj_info(color, 'cylinder', cir_region(cnt[1], cnt[2]), cnt[1], 0.0))
            elif cnt[0] == 'cube':
                res.objects.append(obj_info(color, 'cube', cnt[1], cnt[2], cnt[3]))
            elif cnt[0] == 'cuboid':
                res.objects.append(obj_info(color, 'cuboid', cnt[1], cnt[2], cnt[3]))
            elif cnt[0] == 'noknow':
                res.objects.append(noknow_info(color, cnt[1]))
            elif cnt[0] == 'cube_cuboid':
                for cnt in cnt[1]:
                    if cnt[0] == 'cube':
                        res.objects.append(obj_info(color, 'cube', cnt[1], cnt[2], cnt[3]))
                    elif cnt[0] == 'cuboid':
                        res.objects.append(obj_info(color, 'cuboid', cnt[1], cnt[2], cnt[3]))
                    elif cnt[0] == 'noknow':
                        res.objects.append(noknow_info(color, cnt[1]))
    return res

# def nothing(x):
#     pass #在我们的例子中，函数什么都不做，所以我们简单地通过。


class image_process:

    def __init__(self): 
        rospy.init_node('image_process', anonymous=True)
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((720, 1280, 3), dtype=np.uint8)  # 初始图像
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]]
        self.cnt_img_roi = self.img_roi.copy() # 结果图像
        self.cnt_img = self.img_src.copy() # 结果图像
        self.objects = Img_processResponse()  # 目标存放位置
        self.objects.result = 'failed'
        self.img_num = 0
        self.time_start = time.time()
        self.imgs = np.zeros((5,ROI[1][0]-ROI[0][0],ROI[1][1]-ROI[0][1],3),dtype=np.uint8) # 存储图像
        # 订阅 /camera/color/image_raw 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_sub_callback)
        # 创建一个名为 /image_process/next 的server，注册回调函数 image_srv_callback
        self.image_srv = rospy.Service('/image_process/next', Img_process, self.image_srv_callback)

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]]
        # print self.img_src.dtype, self.img_src.shape  # 720 1280
        # cv2.imshow("image", self.img_src)
        # cv2.imshow("image ROI", self.img_roi)
        self.cnt_img = self.img_src.copy() # 结果图像
        self.cnt_img[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]] = self.cnt_img_roi
        # cv2.imshow("res_image_roi", self.cnt_img_roi)
        cv2.imshow("res_image", self.cnt_img)
        cv2.waitKey(3)

        ############################## TEST
        
        # print(self.imgs.shape)
        if(self.img_num < 5):
            self.imgs[self.img_num] = self.img_roi
            self.img_num += 1
        else:
            self.img_num = 0
            # img_f = np.array(self.imgs, dtype=np.int32)
            img = np.mean(self.imgs, dtype=np.int32, axis=0)
            img = np.array(img, dtype=np.uint8)
            # print(img.shape)
            # cv2.imshow("img", img)
            find_obj = findObject(img)
            self.cnt_img_roi, contours = find_obj.find_object(color=['blue', 'red', 'green'], draw=True, optim=True, approx=True)
            if len(contours) > 0:
                # print(len(contours))
                cnt_img, cnts = find_obj.find_object(color=['blue', 'red', 'green'], draw=True, optim=True, approx=False)
                # print(len(contours))
                # cv2.imshow('find_obj.bgr', find_obj.bgr)
                cv2.imshow('contour', cnt_img)
                cv2.waitKey(3)
                self.time_end = time.time()
                print('total time: {:.5f}s'.format(self.time_end - self.time_start))  # 0.050
                self.time_start = time.time() # 开始时间

                # 打印轮廓
                # print(contours)
                # print_approx_contours(contours)
                self.objects = objects_info(contours)  # 从不同的轮廓信息得到统一格式的目标位置信息
                # print self.objects
        
    def show_hsv_param(self):
        """
        get b g r color region in image
        """
        img = self.img_roi.copy()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        cv2.namedWindow('hsv_image')
        # cv2.createTrackbar('BS','hsv_image',43,255,nothing)
        # cv2.createTrackbar('BV','hsv_image',46,255,nothing)
        # cv2.createTrackbar('GS','hsv_image',43,255,nothing)
        # cv2.createTrackbar('GV','hsv_image',46,255,nothing)
        # cv2.createTrackbar('RS','hsv_image',43,255,nothing)
        # cv2.createTrackbar('RV','hsv_image',46,255,nothing)
        cv2.createTrackbar('GH','hsv_image',77,100,nothing)
        cv2.createTrackbar('GL','hsv_image',35,50,nothing)
        cv2.createTrackbar('BH','hsv_image',124,100,nothing)
        cv2.createTrackbar('BL','hsv_image',100,50,nothing)
        # cv2.createTrackbar('GH','hsv_image',77,100,nothing)
        # cv2.createTrackbar('GL','hsv_image',35,50,nothing)

        bs = 100
        bv = 30
        gs = 43
        gv = 46
        rs = 66
        rv = 66
        gh = 85
        gl = 35

        # bv = cv2.getTrackbarPos('BV','hsv_image')
        # bs = cv2.getTrackbarPos('BS','hsv_image')
        # gs = cv2.getTrackbarPos('GS','hsv_image')
        # gv = cv2.getTrackbarPos('GV','hsv_image')
        # rv = cv2.getTrackbarPos('RV','hsv_image')
        # rs = cv2.getTrackbarPos('RS','hsv_image')
        gh = cv2.getTrackbarPos('GH','hsv_image')
        gl = cv2.getTrackbarPos('GL','hsv_image')
        
    
        # define range of BGR color in HSV
        threshold_blue = np.array([[100,bs,bv], [124,255,255]])
        threshold_green = np.array([[gl,gs,gv], [gh,255,255]])
        threshold_red1 = np.array([[0,rs,rv], [10,255,255]])
        threshold_red2 = np.array([[156,rs,rv], [180,255,255]])
        # Threshold the HSV image to get only BGR colors
        mask_blue = cv2.inRange(hsv, threshold_blue[0], threshold_blue[1])
        mask_green = cv2.inRange(hsv, threshold_green[0], threshold_green[1])
        mask_red1 = cv2.inRange(hsv, threshold_red1[0], threshold_red1[1])
        mask_red2 = cv2.inRange(hsv, threshold_red2[0], threshold_red2[1])
        mask_red = mask_red1 | mask_red2
        # Bitwise-AND mask and original image
        self.blue = cv2.bitwise_and(img, img, mask=mask_blue)
        self.green = cv2.bitwise_and(img, img, mask=mask_green)
        self.red = cv2.bitwise_and(img, img, mask=mask_red)
        rgb_img = np.stack((self.blue[:, :, 0], self.green[:, :, 1], self.red[:, :, 2]), axis=2)
        cv2.imshow('hsv_image', rgb_img)
        cv2.waitKey(3)
        # 返回 bgr 三通道的分量合成的图片
        # return rgb_img
        
    def rgb_thresh(self, img):
        
        cv2.namedWindow('rgb_image')
        # cv2.createTrackbar('R','rgb_image',100,255,nothing)
        # cv2.createTrackbar('G','rgb_image',100,255,nothing)
        cv2.createTrackbar('B','rgb_image',100,255,nothing)

        # r = cv2.getTrackbarPos('R','rgb_image')
        # g = cv2.getTrackbarPos('G','rgb_image')
        b = cv2.getTrackbarPos('B','rgb_image')

        _, thresh = cv2.threshold(img[:,:,0], b, 255, cv2.THRESH_BINARY)

        # define range of BGR 
        # threshold_blue = np.array([[p,0,0], [255,n1,n2]])
        # threshold_green = np.array([[35,gs,gv], [77,255,255]])
        # threshold_red = 
        
        # mask_blue = cv2.inRange(img, threshold_blue[0], threshold_blue[1])
        # mask_green = cv2.inRange(hsv, threshold_green[0], threshold_green[1])
        # mask_red1 = cv2.inRange(hsv, threshold_red1[0], threshold_red1[1])
        # mask_red2 = cv2.inRange(hsv, threshold_red2[0], threshold_red2[1])
        # mask_red = mask_red1 | mask_red2
        # Bitwise-AND mask and original image
        # self.blue = cv2.bitwise_and(img, img, mask=mask_blue)
        # self.green = cv2.bitwise_and(img, img, mask=mask_green)
        # self.red = cv2.bitwise_and(img, img, mask=mask_red)

        cv2.imshow('rgb_image', thresh)
        cv2.waitKey(3)
        
    def image_srv_callback(self, req):
        # 显示请求数据
        print('signal', req.signal)
        if req.signal[0] == 'init':  # init config
            return self.init_config_msg()
        else:
            # 反馈数据
            return self.objects
 
    def init_config_msg(self):
        """ 初始提取放置位置信息 """
        img = self.img_src.copy()
        orange = get_specific_color(img, 'orange') # 提取所需颜色
        gray = cv2.cvtColor(orange, cv2.COLOR_BGR2GRAY)
        gray = gray.astype(np.uint8)
        _, contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # find out contours
        # res = cv2.drawContours(orange, contours, 0, (125, 255, 255), 2) 
        x,y,w,h = cv2.boundingRect(contours[0]) # find bounding rectangle
        # res = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)
        # cv2.imshow('input_image', img)
        # cv2.imshow('res', res)
        # cv2.waitKey(30)
        # print(x,y, x+w/2, y+h/2)

        # 更改ROI
        global ROI 
        ROI = [[int(x-20), int(y-20)], [int(x+w+20), int(y+h+20)]]  # 框选出的区域，在这个区域内进行识别
        print(ROI)
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]]
        self.cnt_img_roi = self.img_roi.copy() # 结果图像
        self.cnt_img = self.img_src.copy() # 结果图像

        # make msg
        obj = Process_Result()
        obj.color = ' '
        obj.type = 'table'
        obj.center = [x+w/2, y+h/2]
        obj.theta = 0.0
        # obj.region = [[x,y], [x+w,y], [x+w,y+h], [x,y+h]]
        obj.region[0].x, obj.region[0].y = x, y
        obj.region[1].x, obj.region[1].y = x+w, y
        obj.region[2].x, obj.region[2].y = x+w, y+h
        obj.region[3].x, obj.region[3].y = x, y+h
        # publish the msg
        res_msg = Img_processResponse()
        res_msg.result = 'init'
        res_msg.objects.append(obj)
        return res_msg
        


if __name__ == '__main__':
    img_proc = image_process()
    

    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



