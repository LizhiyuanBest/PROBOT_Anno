#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

import numpy as np


class VisionManager:
    
    def __init__(self, length, breadth):
        self.table_length = length
        self.table_breadth = breadth
        self.cont = 0

    def get2DLocation(self, img):
        """
        Gets the 2d location of object in camera frame
        return center (x, y)
        """
        self.curr_img = img  # 将图像保存到这个实例的成员中
        # 获得图像中心坐标

        h,w,l = np.shape(img)
        # print(h,w,l)
        self.img_centre_x_ = w / 2
        self.img_centre_y_ = h / 2

        tablePos = self.detectTable() #　找到桌面并计算每mm有多少像素
        # print("tablePos",tablePos)
        pixel_x, pixel_y = self.detect2DObject(tablePos) # 找到物体中心坐标
        # print("(pixel_x, pixel_y) ({},{})".format(pixel_x, pixel_y))
        x,y = self.convertToMM(pixel_x, pixel_y) # 转化到世界坐标系
        # print(x,y)
        return x, y

    def detectTable(self):
        """
        detect2DObject processes the image to isolate object
        """
        # Extract Table from the image and assign values to pixel_per_mm fields
        image = self.curr_img.copy() # 定义一个图像副本
        (b, g, r) = cv2.split(image) # 将图像切分成三份，按照BGR 颜色
        img_median = cv2.medianBlur(r, 3) # 中值滤波
        # 将图像二值化，通过red green颜色
        mask = ((g < 20) & (img_median < 20)).astype(np.uint8)
        mask = mask * 255 # 将符合条件（接近黑色）的像素点变为255，其余的变为0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binaryImage = cv2.dilate(mask,kernel) # 膨胀

        #  Get the centroid of the of the blob
        #  找到桌面的中心
        nonZeroPoints = np.flatnonzero(binaryImage)  # 存储图像中的非零元素的索引到nonZeroPoints中
        box = cv2.boundingRect(binaryImage) # 计算这些非零元素的最小外接矩形　box(247, 146, 306, 306)　(x,y,w,h)
        x,y,w,h = box
        cv2.rectangle(image, (x,y), (x+w,y+h), (0,0,255), 2) # 画出来
        
        #  Update pixels_per_mm fields
        self.pixels_permm_y = h / self.table_length  # 求得y轴缩放倍数
        self.pixels_permm_x = w / self.table_breadth # 求得x轴缩放倍数
        print("Pixels in x {} \nPixels in y {}".format(self.pixels_permm_x, self.pixels_permm_y))


        # cv2.imshow("Table Detection", image)  # 显示
        # cv2.waitKey(5)

        return box # 返回所求的矩形框

    def detect2DObject(self, tablePos):
        """
        detect2DObject processes the image to isolate object
        """
        # Implement Color Thresholding and contour findings to get the location of object to be grasped in 2D
        image = self.curr_img.copy() # 定义一个图像副本
        (b, g, r) = cv2.split(image) # 将图像切分成三份，按照BGR 颜色
        img_median = cv2.medianBlur(g, 3) # 中值滤波

        x,y,w,h = tablePos # 桌子信息
        # 将图像二值化，通过green颜色
        mask = (g > 100).astype(np.uint8)
        mask = mask * 255 # 将符合条件（绿色）的像素点变为255，其余的变为0
        # 桌子外的点清零
        mask[:y+5, :] = 0
        mask[y+h-5:, :] = 0
        mask[:, :x+5] = 0
        mask[:, x+w-5:] = 0

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        binaryImage = cv2.dilate(mask,kernel) # 膨胀
        

        # Get the centroid of the of the blob
        #找到物体中心
        nonZeroPoints = np.flatnonzero(binaryImage)  # 存储图像中的非零元素的索引到nonZeroPoints中
        box = cv2.boundingRect(binaryImage) # 计算这些非零元素的最小外接矩形　box(247, 146, 306, 306)　(x,y,w,h)
        x,y,w,h = box
        cv2.rectangle(image, (x,y), (x+w,y+h), (0,0,255), 2) # 画出来

        pixel_x = x + w / 2 #得到中心坐标
        pixel_y = y + h / 2
        # Test the conversion values
        print("pixel_x in ｘ {} \npixel_y in y {}".format(pixel_x, pixel_y))

        # cv2.imwrite('dect_'+str(self.cont)+'.jpg',image)
        # self.cont += 1
        # cv2.imshow("Centre point", image)  # 显示
        # cv2.waitKey(10)

        return pixel_x, pixel_y

    def convertToMM(self, x, y):
        """
        convertToMM converts pixel measurement to metric
        """
        # Convert from pixel to world co-ordinates in the camera frame
        x = (x - self.img_centre_x_) / self.pixels_permm_x; #物体中心-图像中心
        y = (y - self.img_centre_y_) / self.pixels_permm_y;
        return x, y
    

if __name__ == '__main__':
    img = cv2.imread('/home/li/table.jpg')
    
    vMng = VisionManager(0.3, 0.3)
    
    x,y = vMng.get2DLocation(img)
    print(x,y)