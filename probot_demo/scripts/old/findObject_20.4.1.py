# -*- coding=utf-8 -*-

import cv2
import numpy as np
import math
import copy
import time 


def image_thresh(img):
    """
    将图像二值化，并适当去噪
    """
    ret, thresh = cv2.threshold(img,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv2.imshow('thresh', thresh)
    # noise removal
    kernel = np.ones((3,3),np.uint8)
    opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
    # cv2.imshow('opening', opening)
    return opening


def get_center(contours):  # 求得轮廓中心
    centers = []
    for contour in contours:
        M = cv2.moments(contour)
        # 求重心
        cx = int(M['m10'] / M['m00'])
        cy = int(M['m01'] / M['m00'])
        centers.append([cx, cy])
    return centers


def tri_point_order(pt1, pt0, pt2):
    vec1 = (pt1[0]-pt0[0], pt1[1]-pt0[1])
    vec2 = (pt2[0]-pt0[0], pt2[1]-pt0[1])
    ans = vec1[0]*vec2[1] - vec1[1]*vec2[0]
    # if(ans>0): 
	# 	print ("逆时针" )
    # elif(ans<0):
    #     print ("顺时针" )  
    # elif(ans==0):
	# 	print ("共线") 
    return ans


def calc_distance(pt1, pt2):
    x = pt1[0] - pt2[0]
    y = pt1[1] - pt2[1]
    return math.sqrt(x*x + y*y)


def find_near(contour, a, pts):
    """
    输入一个轮廓，在pts（索引）集合中找到轮廓上离a点（索引）最近的点（索引）
    返回值： pts中离a最近的点（索引）
    """
    this_cnt = tuple(contour[a][0])
    near = -1
    min_dis = float("inf")
    for pt in pts:
        next_cnt = tuple(contour[pt][0])
        distance = calc_distance(this_cnt, next_cnt)
        if distance < min_dis:
            near = pt
            min_dis = distance
    return near


def area_ratio(area1, area2):
    return float(area1) / area2


class findObject:
    """
    一个基于2d轮廓的实例分割
    """
    def __init__(self, image):
        self.img = image  # 原图像
        self.bgr = self.get_BGR_img()  # 得到特定颜色的图像
        self.res_id = 1

    def get_BGR_img(self):
        """
        get b g r color region in image
        """
        img = self.img.copy()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # define range of BGR color in HSV
        threshold_blue = np.array([[100,43,46], [124,255,255]])
        threshold_green = np.array([[35,43,46], [77,255,255]])
        threshold_red1 = np.array([[0,43,46], [10,255,255]])
        threshold_red2 = np.array([[156,43,46], [180,255,255]])
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
        # 返回 bgr 三通道的分量合成的图片
        return np.stack((self.blue[:, :, 0], self.green[:, :, 1], self.red[:, :, 2]), axis=2)

    def draw_contours(self):
        # 绘制轮廓
        res = self.img.copy()  # drawContours 会将轮廓画在输入的图像
        for _, cnts in self.contours.items():
            for i in range(len(cnts)):
                res = cv2.drawContours(res, [cnts[i]], 0, (125, (125+50*i)%255, (125+50*i)%255), 2) 
        return res

    def find_contours(self, gray):
        # 二值化
        mask = image_thresh(gray)
        # 查找轮廓
        mask = mask.astype(np.uint8)  # findContours 输入为单通道图像，类型为uint8
        # 在此轮廓不简化，不用 CHAIN_APPROX_SIMPLE
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # print('contours', len(contours))
        return contours

    def draw_concave(self, image, pts):
        """
        画出凹陷点。pts为凹陷点列表。
        """
        # print('pts', pts)
        for i, pt in enumerate(pts):  # 画出凹陷点
            cv2.circle(image, pt, 3, [0, 100+i*50, 100], -1)
        return image

    def find_concave(self, contour):
        """
        找轮廓的凹陷之处，返回轮廓索引
        """
        flag = False  # 连续凹陷标志
        step = 1  # 距离阈值，相距多远不能有两个凹点
        idx = []
        order_last = 0
        for i in range(0, len(contour)-5, 2):
            # print(contour[i][0], contour[i+1][0], contour[i+2][0])
            order = tri_point_order(contour[i-3][0], contour[i][0], contour[i+3][0])
            # print('order', order)
            if order < -3:  # 凹陷点
                if flag == False: 
                    flag = True
                    idx.append(i)
                    step = 1
                else:
                    if order < order_last: # 更凹的点
                        idx.pop()
                        idx.append(i)
                        step = 1
            elif order > 0:
                step -= 1
                if step <= 0:
                    flag = False
            else:
                step -= 1
            order_last = order
        return idx

    def draw_convex(self, image, convex_point, contour):
        """
        画出凸边
        """
        for i in range(len(convex_point)):
            s, e, f, d = convex_point[i]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            # far = tuple(contour[f][0])
            cv2.line(image, start, end, [0, 100, 255], 2)
        return image

    def find_convex(self, contour):
        """
        找到凸边
        """
        hull = cv2.convexHull(contour, clockwise=True, returnPoints=False) # 找到凸包
        # print(hull)
        defects = cv2.convexityDefects(contour, hull)  # 找到凸缺陷
        # print(defects)
        # 找到凹陷的位置
        convex_point = defects[defects[:, :, 3] > 300]  # 凹陷的阈值 P > 400, N < 222
        # print(convex_point)
        # 将 convex_point 排序 方便后面使用
        convex_point = sorted(convex_point, key=lambda point:point[0],reverse=False)
        convex_point = [x.tolist() for x in convex_point]
        return convex_point        

    def cut_cnt(self, contour, idx):
        """
        将轮廓按照idx一分为二，返回两条轮廓
        """
        idx.sort() # 排序，从小到大
        cnt1_1 = contour[0:idx[0]]
        cnt1_2 = contour[idx[1]:(len(contour)-1)]
        cnt1 = np.vstack((cnt1_1,cnt1_2))  # 第一条轮廓
        cnt2 = contour[idx[0]:idx[1]]  # 第二条轮廓
        return cnt1, cnt2

    def contour_optimization(self, gray, contours):
        """
        输入: 查找轮廓的灰度图和轮廓
        判断每一条轮廓是否为凸，若是，则不处理；
        否则，将轮廓从凹陷处切开，处理为凸轮廓。
        输出: 轮廓
        """
        # print('contours', len(contours))
        # print(contours)
        cnts = []
        res = self.img.copy()
        res = cv2.drawContours(res, contours, -1, (255, 255, 255), 2)
        for contour in contours:  # 遍历轮廓
            # print('contour', contour)
            # print('contour',contour.shape[0])
            hull = cv2.convexHull(contour)  # 求得凸包轮廓
            # print(hull)
            # 求得轮廓面积和凸包面积的比值，以此来判断是否有挨在一起的物体
            solidity = area_ratio(cv2.contourArea(contour), cv2.contourArea(hull))  
            print('solidity', solidity)

            if solidity < 0.92:  # 判断轮廓是否为凸, 轮廓面积与凸包面积之比  P > 0.94, N < 0.9
                idx = self.find_concave(contour)  # 找到凹陷处
                # print('idx', idx)
                # # 画出凹陷点
                # res = self.draw_concave(res, map(lambda i:tuple(contour[i][0]), idx)) 
                # cv2.imshow('res_{}'.format(self.res_id), res)
                if len(idx) <= 1:  # 只有一个点
                    cnts.append(contour)
                    continue
                elif len(idx) == 2:  # 只有两个凹陷点 则将两个点连接起来将轮廓分为两个
                    cnt1, cnt2 = self.cut_cnt(contour, idx) # 将轮廓从凸缺陷处截为两半
                    cnts.append(cnt1)
                    cnts.append(cnt2)
                    continue
                elif len(idx) > 2:
                    convex_point = self.find_convex(contour)
                    # print(convex_point)
                    # # 画出凸包
                    # res = self.draw_convex(res, convex_point, contour)  
                    # cv2.imshow('res_{}'.format(self.res_id), res)
                    # self.res_id += 1
                    # 将检测到的凹陷点与检测的凸包相结合
                    convex_pts = []
                    for convex in convex_point:
                        convex_temp = []
                        for i in idx:
                            if i > convex[0] and i < convex[1]: # 此凹陷点在此凸边内
                                convex_temp.append(i) 
                        convex_pts.append(convex_temp) 
                    # print('convex_pts', convex_pts)
                    # print('len(convex_pts)', len(convex_pts))
                    # 找到两点间连起来是一个物体的，然后递归 将轮廓分割
                    cnt_flag = np.zeros(len(contour), dtype=int)
                    cnt_flag[:] = -1
                    # print('len(contour)', len(contour))
                    # print('len(cnt_flag)', len(cnt_flag))
                    # print('cnt_flag', cnt_flag)
                    for i in range(len(convex_pts)):
                        other_convex = copy.copy(convex_pts)
                        other_convex.remove(other_convex[i]) 
                        other_convex = list(np.concatenate(other_convex))  # flat
                        # print('other_convex', other_convex)
                        for conv in convex_pts[i]:
                            near_idx = find_near(contour, conv, other_convex)
                            print('near_idx', near_idx)
                            cnt_flag[conv] = near_idx
                            if cnt_flag[near_idx] == conv:  # 找到一条轮廓
                                cnt1, cnt2 = self.cut_cnt(contour, [near_idx, conv]) # 将轮廓从凸缺陷处截为两半
                                cnts_t = [cnt1, cnt2]
                                # print('len(cnts_t)', len(cnts_t))
                                for cnt_t in cnts_t:
                                    # 为下一次查找做准备
                                    mask = np.zeros(gray.shape,np.uint8)
                                    # 这里一定要使用参数 -1, 绘制填充的的轮廓
                                    cv2.drawContours(mask,[cnt_t],0,255,-1)
                                    # cv2.imshow('process_gray_{}'.format(res_id), mask)
                                    # cv2.imshow('gray_before{}'.format(res_id), gray)
                                    next_gray = cv2.bitwise_and(gray, gray, mask=mask)
                                    # cv2.imshow('gray_after{}'.format(res_id), gray)
                                    # cv2.imshow('next_gray_{}'.format(res_id), next_gray)
                                    # 查找下一次轮廓
                                    next_contours = self.find_contours(next_gray)
                                    # 优化轮廓 
                                    next_cnts = self.contour_optimization(next_gray, next_contours)
                                    # print('len(next_cnts)', len(next_cnts))
                                    for next_cnt in next_cnts:
                                        cnts.append(next_cnt)
                                break
                        else:  # 加此句，跳出双层for循环
                            continue
                        break
            else:
                cnts.append(contour)
        # print('cnts', cnts)
        return cnts

    def contour_approximate(self, draw=True):
        """
        将轮廓近似为规则形状。
        """
        res = self.img.copy()  # drawContours 会将轮廓画在输入的图像
        for color, cnts in self.contours.items():
            self.cnts_approx[color] = []  # 返回的轮廓
            for contour in cnts:  # 遍历轮廓
                # print('contour', contour)
                area = cv2.contourArea(contour)  # 求得轮廓面积
                (x,y),radius = cv2.minEnclosingCircle(contour)
                solidity = area_ratio(area, 3.141592 * radius * radius) # 轮廓面积 / 圆形面积
                print("area/cir_area ", solidity)
                if solidity > 0.7:  # 认为是圆形  # P>0.845  N<0.64
                    center = (int(x),int(y))
                    radius = int(radius)
                    self.cnts_approx[color].append([center, radius])
                    if draw:
                        res = cv2.circle(res, center, radius, (0,255,255), 2)
                else:  # 否则认为是矩形
                    rect = cv2.minAreaRect(contour)
                    # print('rect', rect)
                    box = cv2.boxPoints(rect)
                    box = box.astype(np.int) 
                    # print('box', box) 
                    solidity = area_ratio(area, cv2.contourArea(box)) # 轮廓面积 / 矩形面积
                    print('area/box_area',solidity)  
                    if solidity > 0.85:  # 认为是矩形  # P>0.92  N<?
                        center = (np.mean(box[:, 0]), np.mean(box[:, 1]))
                        self.cnts_approx[color].append([box.tolist(), center, rect[2]])
                        if draw: # 绘制轮廓
                            res = cv2.drawContours(res, [box], -1, (255, 255, 255), 2)

        return res, self.cnts_approx

    def find_object(self, color=['blue', 'green', 'red'], draw=True, optim=True, approx=True):
        """
        输入： 
        color：tuple，在那些颜色中寻找物体，(blue, green, red).
        draw: 是否将轮廓画出来，如果是，则返回一个画上轮廓的img，否则返回原图片.
        optim: 是否优化轮廓.
        approx：是否将轮廓优化为矩形和圆形，如果是，那么返回坐标，中心，半径或角度.
        返回值：
        图像，轮廓，物体位置
        """
        # print('color', color)
        # 根据颜色查找轮廓，结果放在contours中
        self.contours = {}
        for c in color:
            if c == 'blue':
                self.contours['blue'] = self.find_contours(self.bgr[:, :, 0])  # 蓝色分量 
                if optim: # 优化轮廓
                    self.contours['blue'] = self.contour_optimization(self.bgr[:, :, 0], self.contours['blue'])  # 优化轮廓 
            elif c == 'green':
                self.contours['green'] = self.find_contours(self.bgr[:, :, 1])  # 绿色分量 
                if optim: # 优化轮廓
                    self.contours['green'] = self.contour_optimization(self.bgr[:, :, 1], self.contours['green'])  # 优化轮廓
            elif c == 'red':
                self.contours['red'] = self.find_contours(self.bgr[:, :, 2])  # 红色分量 
                if optim: # 优化轮廓
                    self.contours['red'] = self.contour_optimization(self.bgr[:, :, 2], self.contours['red'])  # 优化轮廓
        # 将轮廓画出来
        res_img = self.img.copy()
        if draw:
            res_img = self.draw_contours()
        # 近似轮廓
        self.cnts_approx = {}
        if approx == True:
            res_img, self.cnts_approx = self.contour_approximate(draw=draw)
        return res_img, self.contours, self.cnts_approx


if __name__ == '__main__':
    
    img = cv2.imread('image_126.jpg')  # 读入图片
    start = time.time() # 开始时间
    # 定义类
    find_obj = findObject(img)
    # cv2.imshow('find_obj.bgr', find_obj.bgr)

    res_cnt, contours, cnts_approx = find_obj.find_object(color=['blue', 'green', 'red'], draw=True, optim=True, approx=True)

    # 打印轮廓数目
    for color, cnts in contours.items():
        print(color, 'contours ', len(cnts))

    # 打印物体数目
    for color, cnts in cnts_approx.items():
        print(color, len(cnts))

    # 打印物体位置
    for color, cnts in cnts_approx.items():
        for cnt in cnts:
            print(color, cnt[-2])
    
    end = time.time()
    print('total time: {:.5f}s'.format(end - start))  # 0.050

    # cv2.imshow('bgr', img)  
    cv2.imshow('res_cnt', res_cnt)
    
    cv2.waitKey(0)