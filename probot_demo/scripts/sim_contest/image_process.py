#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
from findObject import * 
import time 
import rospy
import roslib
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from probot_demo.srv import Img_process, Img_processResponse
from probot_demo.msg import Process_Result
from probot_demo.msg import Pt2D
from hsv import get_specific_color


ROI = [[260, 260], [520, 530]]  # 框选出的区域，在这个区域内进行识别

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


class image_process:

    def __init__(self): 
        self.bridge = CvBridge()  # ros和opencv的桥梁
        self.img_src = np.zeros((800, 800, 3), dtype=np.uint8)  # 初始图像
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]]
        self.cnt_img_roi = self.img_roi.copy() # 结果图像
        self.cnt_img = self.img_src.copy() # 结果图像
        self.objects = Img_processResponse()  # 目标存放位置
        self.objects.result = 'failed'
        # 订阅 /probot_anno/camera/image_raw 话题，原始图像，注册回调函数 image_sub_callback
        self.image_sub = rospy.Subscriber("/probot_anno/camera/image_raw", Image, self.image_sub_callback)
        # 创建一个名为 /image_process/next 的server，注册回调函数 image_srv_callback
        self.image_srv = rospy.Service('/image_process/next', Img_process, self.image_srv_callback)

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        # 更新图像
        # 通过cv_bridge 得到opencv格式的image
        self.img_src = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.img_roi = self.img_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]]
        # print self.img.dtype, self.img.shape
        cv2.imshow("image", self.img_src)
        # cv2.imshow("image ROI", self.img_roi)
        self.cnt_img = self.img_src.copy() # 结果图像
        self.cnt_img[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]] = self.cnt_img_roi
        # cv2.imshow("res_image_roi", self.cnt_img_roi)
        cv2.imshow("res_image", self.cnt_img)
        cv2.waitKey(3)
        
    def image_srv_callback(self, req):
        # 显示请求数据
        print('signal', req.signal)
        if req.signal[0] == 'init':  # init config
            return self.init_config_msg()
        else:
            start = time.time() # 开始时间
            # try:
            # 定义类
            find_obj = findObject(self.img_roi)
            # cv2.imshow('find_obj.bgr', find_obj.bgr)

            self.cnt_img_roi, contours = find_obj.find_object(color=req.signal, draw=True, optim=True, approx=True)
            # self.cnt_img = self.img_src.copy() # 结果图像
            # self.cnt_img[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]] = self.cnt_img_roi
            # # cv2.imshow("res_image_roi", self.cnt_img_roi)
            # cv2.imshow("res_image", self.cnt_img)
            # cv2.waitKey(3)
            # print contours
            # 打印轮廓数目
            print_approx_contours(contours)
            self.objects = objects_info(contours)  # 从不同的轮廓信息得到统一格式的目标位置信息
            
            end = time.time()
            print('total time: {:.5f}s'.format(end - start))  # 0.050
            # print self.objects
            # except:
            #     pass
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
        
    def test(self):
        start = time.time() # 开始时间
        # 定义类
        find_obj = findObject(self.img_roi)
        # cv2.imshow('find_obj.bgr', find_obj.bgr)

        self.cnt_img_roi, contours = find_obj.find_object(color=['blue','green','red'], draw=True, optim=True, approx=True)
        print contours
        # 打印轮廓数目
        print_approx_contours(contours)
        self.objects = objects_info(contours)  # 从不同的轮廓信息得到统一格式的目标位置信息
        print self.objects
        
        end = time.time()
        print('total time: {:.5f}s'.format(end - start))  # 0.050

if __name__ == '__main__':
    img_proc = image_process()
    rospy.init_node('image_process', anonymous=True)
    # for _ in range(10):  # 测试用
    # img_proc.test()
    # cv2.waitKey(0)
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


# 找到初始放置位置
# if __name__ == '__main__':
#     img = cv2.imread('./image_151.jpg',-1)
#     orange = get_specific_color(img, 'orange')
#     gray = cv2.cvtColor(orange, cv2.COLOR_BGR2GRAY)
#     gray = gray.astype(np.uint8)
#     _, contours, _ = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
#     res = cv2.drawContours(orange, contours, 0, (125, 255, 255), 2) 
#     x,y,w,h = cv2.boundingRect(contours[0])
#     res = cv2.rectangle(img, (x,y), (x+w,y+h), (0,255,0), 2)

#     print(x,y, x+w/2, y+h/2)
#     # cv2.imshow('input_image', img)
#     # cv2.imshow('orange', orange)
#     cv2.imshow('res', res)
#     cv2.waitKey(0)


# 截取模板
# if __name__ == '__main__':
#     print('Hello Python')
#     img = cv2.imread('./image_150.jpg')
#     image = img[230:500, 260:530, :]
#     cv2.imwrite('red_cube.jpg', image)
#     # img[320:, :, :] = 0
#     # img[:, 416:, :] = 0
#     cv2.imshow('input_image', image)
#     cv2.waitKey(0)
    


#     cv2.destroyAllWindows()
