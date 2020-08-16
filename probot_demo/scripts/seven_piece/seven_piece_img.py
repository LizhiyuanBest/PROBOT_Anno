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

TEST = False
# TEST = True

pi = 3.14159
# ROI = [[240, 400], [620, 920]]  # 框选出的区域，在这个区域内进行识别
ROI = [[205, 375], [565, 860]]  # 框选出的区域，在这个区域内进行识别

# color pens
pens = [(50,0,50),(50,100,150),(150,0,50),(50,100,50),(100,200,50),(200,200,200),(150,100,200)]

# color threshold
hsv_th = {'blue' : [[70,43,43], [125,255,255]], \
        'green'  : [[26,43,46], [50,255,200]],  \
        'red'    : [[0,100,85], [3,255,255], [170,100,100], [180,255,255]], \
        'yellow' : [[18,160,46], [26,255,255]], \
        'orange' : [[5,150,140], [13,255,255]],   \
        'purple' : [[125,50,50], [170,255,255]],\
        'white'  : [[0,0,135], [180,35,255]]}

def get_specific_color(img, color):
    """
    get specific color region in rgb image
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if color == 'red':
        # define threshold range of specific color in HSV
        lower_red1 = np.array(hsv_th[color][0])
        upper_red1 = np.array(hsv_th[color][1])
        lower_red2 = np.array(hsv_th[color][2])
        upper_red2 = np.array(hsv_th[color][3])
        # Threshold the HSV image to get only specific colors
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2
        # Bitwise-AND mask and original image
        return cv2.bitwise_and(img, img, mask=mask_red)
    else:
        # define threshold range of specific color in HSV
        lower_th = np.array(hsv_th[color][0])  
        upper_th = np.array(hsv_th[color][1])
        # Threshold the HSV image to get only specific colors
        mask = cv2.inRange(hsv, lower_th, upper_th)
        # Bitwise-AND mask and original image
        return cv2.bitwise_and(img, img, mask=mask)

def nothing(x):
    pass #在我们的例子中，函数什么都不做，所以我们简单地通过。

def test_specific_color(img):
    """
    test specific color region in rgb image
    """
    # Convert BGR to HSV
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    cv2.namedWindow('hsv_image')  # 调参用
    cv2.createTrackbar('HL','hsv_image',70,180,nothing)
    cv2.createTrackbar('HH','hsv_image',125,180,nothing)
    cv2.createTrackbar('SL','hsv_image',43,255,nothing)
    cv2.createTrackbar('SH','hsv_image',255,255,nothing)
    cv2.createTrackbar('VL','hsv_image',43,255,nothing)
    cv2.createTrackbar('VH','hsv_image',255,255,nothing)
    
    hl = cv2.getTrackbarPos('HL','hsv_image')
    hh = cv2.getTrackbarPos('HH','hsv_image')
    sl = cv2.getTrackbarPos('SL','hsv_image')
    sh = cv2.getTrackbarPos('SH','hsv_image')
    vl = cv2.getTrackbarPos('VL','hsv_image')
    vh = cv2.getTrackbarPos('VH','hsv_image')

    # define threshold range of color in HSV
    lower_th = np.array([hl,sl,vl])  
    upper_th = np.array([hh,sh,vh]) 
    # Threshold the HSV image to get only specific colors
    mask = cv2.inRange(hsv, lower_th, upper_th)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(img, img, mask=mask)
    cv2.imshow('hsv_image', res)
    cv2.waitKey(3)

def img_filtering(img):
    """
    image filtering 
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
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

def calc_distance(pt1, pt2):
    """ calc the distance of two poiints """
    x = pt1[0] - pt2[0]
    y = pt1[1] - pt2[1]
    return math.sqrt(x*x + y*y)

def long_edge(cnt):
    """ calculate the longest eage in triangle or quadrilateral """
    # print(cnt)
    len_cnt = len(cnt)
    try:
        max_dis = 0
        max_dis_pts = []
        other_pts = []
        for i in range(len_cnt-1):
            for j in range(i,len_cnt,1):
                dis = calc_distance(cnt[i], cnt[j])
                if(dis > max_dis):
                    max_dis = dis
                    max_dis_pts = [cnt[i], cnt[j]]
                    other_pts = []
                    for pt in cnt:
                        if pt not in max_dis_pts:
                            other_pts.append(pt)
        return max_dis_pts, other_pts
    except:
        print('error in calcing longest edge')

def calc_line_angle(pt1, pt2):
    """ calc angle of line using two points """
    x = pt1[0] - pt2[0]
    y = pt1[1] - pt2[1]
    return math.atan(float(y)/x)

def point_line_order(line, pt):
    """ calc point over line ? line is two point form """
    # (y-y2)/(y1-y2) - (x-x2)/(x1-x2) = 0
    order = (pt[1]-line[1][1])/(line[0][1]-line[1][1]) \
            - (pt[0]-line[1][0])/(line[0][0]-line[1][0])
    return np.sign(order)

def obj_info(color, obj_type, region, cent, angle):
    """  package information """
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
        self.type_color = ['blue', 'green', 'red', 'yellow', 'orange', 'purple', 'white']
        self.pens = {}
        for i,t in enumerate(self.type_color) :
            self.pens[t] = pens[i]
        self.cnts = {}
        self.objs = []
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
        if(self.img_num < 5): # 多副图像去噪
            self.imgs[self.img_num] = self.img_roi
            self.img_num += 1
        else:
            self.img_num = 0
            img = np.mean(self.imgs, dtype=np.int32, axis=0)
            self.avg_roi = np.array(img, dtype=np.uint8)

            if TEST:
                test_specific_color(self.avg_roi) # 蓝色区域mask
                return
            # #######################
            # get objs' pose
            self.cnt_img_roi, objs = self.get_objs(self.avg_roi)
            # package information
            self.objects = self.objects_info(objs)
            # print(self.objects)
            # show the image
            cv2.waitKey(3)
            # calc cost time 
            self.time_end = time.time()
            print('total time: {:.5f}s'.format(self.time_end - self.time_start))  # 0.050
            self.time_start = time.time() # 开始时间

    def draw_contours(self, img, contours, color=(0,255,0)):
        """ draw contours in img """
        img = cv2.drawContours(img, contours, -1, color, 5)
        return img

    def draw_contour(self, img, contour, color=(0,255,0)):
        """ draw one contour in img """
        img = cv2.drawContours(img, [contour], 0, color, 3)
        return img

    def find_contours(self, mask):
        """ find contours in mask """
        mask = mask.astype(np.uint8)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) # CHAIN_APPROX_NONE
        return contours

    def find_max_contour(self, mask):
        """ find max contour in mask """
        mask = mask.astype(np.uint8)
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        return self.max_contour(contours)

    def max_contour(self, contours):
        """ return the max contour """
        if len(contours) == 0:
            return []
        else:
            max_cnt = []
            max_area = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                # print(area)
                if area > 1000 and area > max_area:
                    max_area = area
                    max_cnt = cnt
            return max_cnt

    def find_convex(self, contour):
        """
        将轮廓近似为多边形，返回优化后的轮廓
        """
        epsilon = 0.03*cv2.arcLength(contour,True)
        cnt = cv2.approxPolyDP(contour,epsilon,True)
        # print("approx", cnt)
        return cnt

    def flatten_convexs(self, convexs):
        """ 提取轮廓上的顶点 """
        cnt = []
        for pt in convexs:
            cnt.append(list(pt[0]))
        return cnt

    def objects_info(self, objs):
        """ 统一格式的目标位置信息 """
        res = Img_processResponse()
        if objs == []: # 没有物体
            res.result = 'faild'
            print('no objs')
            return res
        # 找到物体
        res.result = 'ok'
        for obj in objs:
            res.objects.append(obj_info(obj[0], obj[1], obj[2], obj[3], obj[4]))
        return res

    def get_objs(self, img_roi):
        """ calc objs' pose """
        # get specific colors
        img = img_roi.copy()
        # image = img_roi.copy()
        objs = [] # 
        for color in self.type_color:
            # get specific color in type_color
            specific_color = get_specific_color(self.avg_roi, color) 
            # cv2.imshow(color+'1', specific_color)
            # filtering the specific color
            specific_color = img_filtering(specific_color)
            # show the specific color
            # cv2.imshow(color, specific_color)
            # find contours
            contour = self.find_max_contour(specific_color)
            # print(contour)
            if contour == []:
                continue
            # draw contours
            img = self.draw_contour(img, contour, color=self.pens[color])
            # find polygon vertex
            convexs = self.find_convex(contour)
            vertexs = self.flatten_convexs(convexs)
            # draw vertex
            # image = self.draw_contours(image, convexs, color=self.pens[color])
            # calc objs' information
            print(color)
            if len(vertexs) == 3: # triangle
                obj_type, center, ang = self.triangle(vertexs)
                print(center, ang)
                # save the objs' information
                objs.append([color, obj_type, vertexs, center, ang])
            elif len(vertexs) == 4: # quadrilateral
                obj_type, center, ang = self.quadrilateral(vertexs)
                print(center, ang)
                # save the objs' information
                objs.append([color, obj_type, vertexs, center, ang])
            else:
                print('error in calc vertexs')

        # cv2.imshow('contours', img)
        # cv2.imshow('points', image)
        # cv2.waitKey(3)
        return img, objs

    def image_srv_callback(self, req):
        # 显示请求数据
        # print('signal', req.signal)
        # 反馈数据
        return self.objects

    def triangle(self, vertexs):
        """ calc position and angle of triangle in plane """
        # calc angle
        l_edge, exclude_pt = long_edge(vertexs)
        # print(l_edge)
        # print(exclude_pt)
        order = point_line_order(l_edge, exclude_pt[0])
        # print(order)
        ang = -calc_line_angle(l_edge[0], l_edge[1])
        ang = ang/pi*180
        # print(ang)
        if order == -1:
            ang += 180
            if(ang>180):
                ang -= 360
        # calc center
        vertexs = np.matrix(vertexs)
        center = [np.mean(vertexs[:, 0]), np.mean(vertexs[:, 1])]
        # print(center)
        return 'triangle', center, ang

    def quadrilateral(self, vertexs):
        """ calc position and angle of quadrilateral in plane """
        # calc center
        points = np.matrix(vertexs)
        center = [np.mean(points[:, 0]), np.mean(points[:, 1])]
        # print(center)
        # calc angle
        l_edge, exclude_pt = long_edge(vertexs)
        # print(l_edge)
        # print(exclude_pt)
        # calc the distance of the shortest and the longest edge
        l_dis = calc_distance(l_edge[0], l_edge[1])
        s_dis = calc_distance(exclude_pt[0], exclude_pt[1])
        # print(l_dis-s_dis) 
        if (l_dis-s_dis) < 20: # square
            ang = -calc_line_angle(l_edge[0], l_edge[1])
            ang = ang/pi*180
            if ang < -45:
                ang += 90
            elif ang > 45:
                ang -= 45
            return 'square', center, ang
        else: # parallelogram
            ang = -calc_line_angle(l_edge[0], l_edge[1])
            ang = ang/pi*180
            return 'parallelogram', center, ang
    
    def test_img(self, file):
        """ test in outline """
        image_src = cv2.imread(file)
        # cv2.imshow('src', image_src)
        self.time_start = time.time() # starting time
        self.avg_roi = image_src[ROI[0][0]:ROI[1][0], ROI[0][1]:ROI[1][1]] # ROI
        # cv2.imshow('ROI', self.avg_roi)
        # get objs' pose
        self.cnt_img_roi, objs = self.get_objs(self.avg_roi)
        # package information
        self.objects = self.objects_info(objs)
        # print(self.objects)
        # calc cost time 
        self.time_end = time.time()
        print('total time: {:.5f}s'.format(self.time_end - self.time_start))  # 0.118
        # show the image
        cv2.waitKey(0)
        

if __name__ == '__main__':
    img_proc = image_process()
    # outline 
    # img_proc.test_img("/home/li/ROS/probot_ws/src/PROBOT_Anno/probot_demo/piece/image_216.jpg")
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()



