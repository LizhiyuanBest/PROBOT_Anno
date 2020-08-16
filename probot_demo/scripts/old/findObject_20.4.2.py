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
    blur = cv2.blur(img,(3,3)) # 中值滤波
    # cv2.imshow('blur', blur)
    _, thresh = cv2.threshold(blur,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    # cv2.imshow('thresh', thresh)    
    # kernel = np.ones((3,3),np.uint8)  # 开运算会一定程度上减弱边缘，但是裁剪后的轮廓最好是加一下这个，可以滤出一下不需要的信息
    # opening = cv2.morphologyEx(thresh,cv2.MORPH_OPEN,kernel, iterations = 2)
    # cv2.imshow('opening', opening)  
    # return opening
    return thresh


def circle_fit(contour):
    """
    最小二成法拟合圆。
    """
    cnts = [tuple(cnt[0]) for cnt in contour]
    # print('cnts', cnts)
    x = np.array([cnt[0] for cnt in cnts])
    y = np.array([cnt[1] for cnt in cnts])
    # print('contour', x, y)
    n=len(x)
    xx=x**2
    yy=y**2
    xy=x*y
    A=np.array([[sum(x),sum(y),n],[sum(xy),sum(yy),sum(y)],[sum(xx),sum(xy),sum(x)]])
    B=np.array([-sum(xx + yy),-sum(xx*y + yy*y),-sum(xx*x +xy*y)]).transpose()
    a=np.linalg.solve(A,B)
    xc=-.5*a[0]
    yc=-.5*a[1]
    r=np.sqrt((a[0]**2+a[1]**2)/4-a[2])
    xc = np.int(xc)
    yc = np.int(yc)
    r = np.int(r)
    # print('xc=%d,yc=%d,r=%d'%(xc,yc,r))
    return xc,yc,r


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
    return float(area1) / (area2 + 1e-10)


def convex_concave_joint(convex_point, concave_idx):
    """ 将检测到的凹陷点与检测的凸包相结合 """ 
    convex_pts = []
    for convex in convex_point:
        convex_temp = []
        for i in concave_idx:
            if i > convex[0] and i < convex[1]: # 此凹陷点在此凸边内
                convex_temp.append(i) 
        convex_pts.append(convex_temp) 
    return convex_pts


def list_sort(l, k=0, r=0):
    # 列表按照k列排序，返回前r个
    l =  sorted(l, key=lambda pt: pt[k], reverse=True)
    if r <= 0:
        return l
    else:
        return l[0:r]


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
        step = 3  # 距离阈值，相距多远不能有两个凹点
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
                    step = 3
                else:
                    if order < order_last: # 更凹的点
                        idx.pop()
                        idx.append(i)
                        step = 3
                order_last = order
            elif order > 0:
                step -= 1
                if step <= 0:
                    flag = False
                    order_last = order
            else:
                step -= 1
                if step <= -3:
                    flag = False
                    order_last = order
        return idx

    def draw_convex(self, image, convex_point, contour):
        """
        画出凸边
        """
        for i in range(len(convex_point)):
            s, e, _, _ = convex_point[i]
            start = tuple(contour[s][0])
            end = tuple(contour[e][0])
            # far = tuple(contour[f][0])
            # distance = tuple(contour[d][0])
            cv2.line(image, start, end, [0, 100, 255], 2)
        return image

    def find_convex(self, contour, convex_threshold=500):
        """
        找到凸边
        """
        hull = cv2.convexHull(contour, clockwise=True, returnPoints=False) # 找到凸包
        # print(hull)
        defects = cv2.convexityDefects(contour, hull)  # 找到凸缺陷
        # print('defects', defects)
        # 找到凹陷的位置
        convex_point = defects[defects[:, :, 3] > convex_threshold]  # 凹陷的阈值 P > 400, N < 222
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

    def draw_approx_contours(self):
        # 绘制轮廓
        res = self.img.copy()  # drawContours 会将轮廓画在输入的图像
        for _, cnts in self.contours.items():
            # print('cnts', cnts)
            for cnt in cnts:
                # print('cnt', cnt)
                if cnt[0] == 'cylinder':
                    res = cv2.circle(res, cnt[1], cnt[2], (0,255,255), 2)
                elif cnt[0] == 'cube':
                    res = cv2.drawContours(res, np.array([cnt[1]]), -1, (255, 0, 255), 2)
                elif cnt[0] == 'cuboid':
                    res = cv2.drawContours(res, np.array([cnt[1]]), -1, (255, 255, 0), 2)
                elif cnt[0] == 'noknow':
                    res = cv2.drawContours(res, cnt[1], -1, (255, 255, 255), 2)
                elif cnt[0] == 'cube_cuboid':
                    for cnt_1 in cnt[1]:
                        if cnt_1[0] == 'cube':
                            res = cv2.drawContours(res, np.array([cnt_1[1]]), -1, (255, 0, 255), 2)
                        elif cnt_1[0] == 'cuboid':
                            res = cv2.drawContours(res, np.array([cnt_1[1]]), -1, (255, 255, 0), 2)
                        elif cnt[0] == 'noknow':
                            res = cv2.drawContours(res, cnt_1[1], -1, (255, 255, 255), 2)
        return res

    def contour_approximate(self, contour):
        """
        将轮廓近似为规则形状。
        """
        # print('contour', contour)
        area = cv2.contourArea(contour)  # 求得轮廓面积
        (x,y),radius = cv2.minEnclosingCircle(contour)
        # print((x,y),radius)
        solidity = area_ratio(area, 3.141592 * radius * radius) # 轮廓面积 / 圆形面积
        print('area/cir_area  {}'.format(solidity))
        if solidity > 0.7:  # 认为是圆形  # P>0.845  N<0.64
            x, y, radius = circle_fit(contour)  # 圆拟合
            # print('xc={},xy={},R={}'.format(x,y,radius))
            fit_cir_ratio = area_ratio(3.141592 * radius * radius, area) # 轮廓面积 / 圆形面积
            print('fit_cir_ratio', fit_cir_ratio)
            if solidity > 0.8:
                center = (int(x),int(y))
                radius = int(radius)
                # return [center, radius]
                return ['cylinder', center, radius]
            else:
                rect = cv2.minAreaRect(contour)
                # print('rect', rect)
                box = cv2.boxPoints(rect)
                box = box.astype(np.int) 
                # print('box', box) 
                ratio = area_ratio(area, cv2.contourArea(box)) # 轮廓面积 / 矩形面积
                print('area/box_area ratio',ratio) 
                if ratio > 0.85:  # 认为是矩形  # P>0.9  N<?
                    center = (np.mean(box[:, 0]), np.mean(box[:, 1]))
                    wh = list(rect[1])
                    wh.sort()
                    # print("wh",wh,area_ratio(wh[0], wh[1]))
                    if area_ratio(wh[0], wh[1]) >0.8:  # 认为是正方形  # P>0.95  N<55
                        return ['cube', box.tolist(), center, rect[2]]
                    else: 
                        return ['cuboid', box.tolist(), center, rect[2]]
                else:
                    center = (int(x),int(y))
                    radius = int(radius)
                    # return [center, radius]
                    return ['cylinder', center, radius]
        else:  # 否则判断是否是矩形
            rect = cv2.minAreaRect(contour)
            # print('rect', rect)
            box = cv2.boxPoints(rect)
            box = box.astype(np.int) 
            # print('box', box) 
            solidity = area_ratio(area, cv2.contourArea(box)) # 轮廓面积 / 矩形面积
            print('area/box_area',solidity)  
            if solidity > 0.8:  # 认为是矩形  # P>0.84  N<?
                center = (np.mean(box[:, 0]), np.mean(box[:, 1]))
                wh = list(rect[1])
                wh.sort()
                ratio = area_ratio(wh[0], wh[1])
                # print("wh", wh, ratio)
                if ratio > 0.8:  # 认为是正方形  # P>0.95  N<0.55
                    return ['cube', box.tolist(), center, rect[2]]
                elif ratio > 0.45: # 长方形
                    return ['cuboid', box.tolist(), center, rect[2]]
                else: # 这应该是长方体正方体连在一起误判为长方形的情况
                    # 检测凸边
                    convex_point = self.find_convex(contour, convex_threshold=100)
                    # print(convex_point)
                    if len(convex_point) < 2:
                        return ['noknow', contour]
                    if len(convex_point) == 2:
                        idx = [convex_point[0][2], convex_point[1][2]]
                    elif len(convex_point) > 2: # 只取前两个值
                        convex_point = list_sort(convex_point, k=3, r=2)
                        # print(convex_point)
                        convex_point =  sorted(convex_point, key=lambda pt: pt[2], reverse=False)
                        # print(convex_point)
                        idx = [convex_point[0][2], convex_point[1][2]]
                    cnt1, cnt2 = self.cut_cnt(contour, idx)
                    cnt1 = self.contour_approximate(cnt1)
                    cnt2 = self.contour_approximate(cnt2)
                    if cnt1[0] == 'noknow' and cnt2[0] == 'noknow':
                        return ['noknow', contour]
                    return ['cube_cuboid', [cnt1, cnt2]]
            else: # 不是矩形和圆形，忽略此轮廓
                return ['noknow', contour]

    def contour_optimization(self, gray, contours, convex_threshold=500, approx=False):
        """
        输入: 查找轮廓的灰度图和轮廓
        判断每一条轮廓是否为凸，若是，则不处理；
        否则，将轮廓从凹陷处切开，处理为凸轮廓。
        approx：是否将轮廓优化为矩形和圆形，如果是，那么返回坐标，中心，半径或角度，否则返回轮廓.
        输出: 轮廓
        """
        conv_th = convex_threshold  # 凸边阈值
        # print('contours', len(contours))
        # print(contours)
        cnts = []
        # res = self.img.copy() # 显示中间结果用的，凹陷点，凸边
        # res = cv2.drawContours(res, contours, -1, (255, 255, 255), 2)
        for contour in contours:  # 遍历轮廓
            # print('contour', contour)
            idx = self.find_concave(contour)  # 找到凹陷处
            # print('idx', idx)
            # 画出凹陷点
            # res = self.draw_concave(res, map(lambda i:tuple(contour[i][0]), idx)) 
            # cv2.imshow('res_{}'.format(self.res_id), res)
            # self.res_id += 1
            if len(idx) == 0:  # 没有凹陷点，是全凸的，一个轮廓
                if approx:
                    contour = self.contour_approximate(contour)
                    # print('coutour_0',contour)
                cnts.append(contour)
                continue
            # 检测凸边
            convex_point = self.find_convex(contour, convex_threshold=conv_th)
            # print(convex_point)
            # 画出凸包
            # res = self.draw_convex(res, convex_point, contour)  
            # cv2.imshow('res_{}'.format(self.res_id), res)
            # self.res_id += 1
            # 将检测到的凹陷点与检测的凸包相结合
            convex_pts = convex_concave_joint(convex_point, idx)
            # print('convex_pts_{}'.format(self.res_id), convex_pts)
            # print('len(convex_pts_{}'.format(self.res_id), len(convex_pts))
            if len(idx) == 1:  # 只有一个点
                if approx:
                    contour = self.contour_approximate(contour)
                    # print('coutour_1',contour)
                cnts.append(contour)
                continue
            elif len(idx) == 2:  # 只有两个凹陷点 则将两个点连接起来将轮廓分为两个
                if len(convex_pts) == 2:
                    cnt1, cnt2 = self.cut_cnt(contour, idx) # 将轮廓从凸缺陷处截为两半
                    if approx:
                        cnt1 = self.contour_approximate(cnt1)
                        # print('cnt2_1',cnt1)
                        cnt2 = self.contour_approximate(cnt2)
                        # print('cnt2_2',cnt2)
                        if cnt1[0] == 'noknow' and cnt2[0] == 'noknow':
                            cnts.append(['noknow', contour])
                            continue
                    cnts.append(cnt1)
                    cnts.append(cnt2)
                else:
                    if approx:
                        contour = self.contour_approximate(contour)
                        # print('contour2_3',contour)
                    cnts.append(contour)
                continue
            elif len(idx) > 2:
                if len(convex_pts) < 2: # 说明凹点和凸边之间的对应关系有问题,调整一下
                    conv_th = conv_th/2 # 凸边阈值更新
                    # 检测凸边
                    convex_point = self.find_convex(contour, convex_threshold=conv_th)
                    # print(convex_point)
                    # 画出凸包
                    # res = self.draw_convex(res, convex_point, contour)  
                    # cv2.imshow('res_{}'.format(self.res_id), res)
                    # self.res_id += 1
                    # 将检测到的凹陷点与检测的凸包相结合
                    convex_pts = convex_concave_joint(convex_point, idx)
                    # print('convex_pts_{}'.format(self.res_id), convex_pts)
                    # print('len(convex_pts_{}'.format(self.res_id), len(convex_pts))
                if len(convex_pts) >= 2:
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
                            # print('near_idx', near_idx)
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
                                    next_cnts = self.contour_optimization(
                                        next_gray, next_contours, convex_threshold=conv_th, approx=approx)
                                    # print('len(next_cnts)', len(next_cnts))
                                    for next_cnt in next_cnts:
                                        cnts.append(next_cnt)
                                break
                        else:  # 加此句，跳出双层for循环
                            continue
                        break
        # print('cnts', cnts)
        return cnts

    def find_object(self, color=['blue', 'green', 'red'], draw=True, optim=True, approx=True):
        """
        输入： 
        color：tuple，在那些颜色中寻找物体，(blue, green, red).
        draw: 是否将轮廓画出来，如果是，则返回一个画上轮廓的img，否则返回原图片.
        optim: 是否优化轮廓.
        approx：是否将轮廓优化为矩形和圆形，如果是，那么返回坐标，中心，半径或角度，否则返回轮廓.
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
                    self.contours['blue'] = self.contour_optimization(self.bgr[:, :, 0], self.contours['blue'], approx=approx)  # 优化轮廓 
            elif c == 'green':
                self.contours['green'] = self.find_contours(self.bgr[:, :, 1])  # 绿色分量 
                if optim: # 优化轮廓
                    self.contours['green'] = self.contour_optimization(self.bgr[:, :, 1], self.contours['green'], approx=approx)  # 优化轮廓
            elif c == 'red':
                self.contours['red'] = self.find_contours(self.bgr[:, :, 2])  # 红色分量 
                if optim: # 优化轮廓
                    self.contours['red'] = self.contour_optimization(self.bgr[:, :, 2], self.contours['red'], approx=approx)  # 优化轮廓
        # 将轮廓画出来
        res_img = self.img.copy()
        if draw:
            if approx == True:
                res_img = self.draw_approx_contours()
            else:
                res_img = self.draw_contours()

        return res_img, self.contours


# def print_approx_contours(contours):
#     """ 打印优化后的轮廓信息，优化前的不要调用此方法 """
#     for color, cnts in contours.items():
#         # print('cnts', cnts)
#         for cnt in cnts:
#             # print('cnt', cnt)
#             if cnt[0] == 'cylinder':
#                 print('{} cylinder: center={}, R={}'.format(color,cnt[1], cnt[2]))
#             elif cnt[0] == 'cube':
#                 print('{} cube: center={}, theta={}'.format(color,cnt[2], cnt[3]))
#             elif cnt[0] == 'cuboid':
#                 print('{} cuboid: center={}, theta={}'.format(color,cnt[2], cnt[3]))
#             elif cnt[0] == 'noknow':
#                 print('{} noknow: center={}'.format(color, get_center([cnt[1]])))
#             elif cnt[0] == 'cube_cuboid':
#                 for cnt_1 in cnt[1]:
#                     if cnt_1[0] == 'cube':
#                         print('{} cube: center={}, theta={}'.format(color,cnt[2], cnt[3]))
#                     elif cnt_1[0] == 'cuboid':
#                         print('{} cuboid: center={}, theta={}'.format(color,cnt[2], cnt[3]))
#                     elif cnt[0] == 'noknow':
#                         print('{} noknow: center={}'.format(color, get_center([cnt_1[1]])))


if __name__ == '__main__':
    
    img = cv2.imread('image_136.jpg')  # 读入图片
    start = time.time() # 开始时间
    # 定义类
    find_obj = findObject(img)
    # cv2.imshow('find_obj.bgr', find_obj.bgr)

    res_cnt, contours = find_obj.find_object(color=['blue', 'green', 'red'], draw=True, optim=True, approx=True)

    # 打印轮廓数目
    # find_obj.print_approx_contours()

        
    # # 打印物体数目
    # for color, cnts in contours.items():
    #     print(color, len(cnts))

    # # 打印物体位置
    # for color, cnts in contours.items():
    #     for cnt in cnts:
    #         print(color, cnt[-2])
    
    end = time.time()
    print('total time: {:.5f}s'.format(end - start))  # 0.050

    # cv2.imshow('bgr', img)  
    cv2.imshow('res_cnt', res_cnt)
    # cv2.imshow('res_approx', res_approx)
    
    cv2.waitKey(0)