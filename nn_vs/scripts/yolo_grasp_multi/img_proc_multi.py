#!/home/li/anaconda3/bin python3
# -*- coding: utf-8 -*-

# python3 ''

import rospy
import copy
import os
import random
from sensor_msgs.msg import Image as sensor_Image
from nn_vs.srv import yolo_grasp, yolo_graspResponse
import copy
import numpy as np
import PIL
import math
import time
from os.path import join
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
sys.path.append('/home/li/visual_servo/yolov3_grasp')
from models import *
from project.utils import *
from project.datasets import *
import project.torch_utils as torch_utils
from torchvision import transforms as T

pi = 3.14159

# dx = dy = float(400) / 290  # mm/像素  1.37931
dx = float(300)/(560-176)
dy = float(300)/(418-35)

u0 = 240  
v0 = 320  

Images_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images/image_200.jpg'
yolo_path = '/home/li/visual_servo/yolov3_grasp_multi'
cfg = join(yolo_path, 'cfg/yolov3-tiny.cfg')
data_cfg = join(yolo_path, 'data/objs.data')
weights = join(yolo_path, 'weights/best.pt')

fourcc = 'mp4v'
img_size = 416
conf_thres = 0.5
nms_thres = 0.5

device = torch_utils.select_device()
model = Darknet(cfg, img_size)
# Load weights
if weights.endswith('.pt'):  # pytorch format
    model.load_state_dict(torch.load(weights, map_location=device)['model'])

# Fuse Conv2d + BatchNorm2d layers
model.fuse()
# Eval mode
model.to(device).eval()

 # Get classes and colors
classes = ['bolt', 'pipe']
colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(classes))]


global detect_flag
detect_flag = False
Yaw = 0.0
L = 128
L2 = L//2
transform = T.ToTensor()

class Image_Processor:

    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('Image_Processor', anonymous=True)

        self.img_t = cv2.imread(Images_PATH)
        self.img_c = None
        self.ang = 0.0
        self.x = v0
        self.y = u0

        self.image_sub = rospy.Subscriber("/probot_anno/hand_camera/hand_image_raw",sensor_Image,self.image_sub_callback)
        self.image_srv = rospy.Service('/detect/server', yolo_grasp, self.detectCallback)
        print('waitting for request.')

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        start = time.time()
        global detect_flag
        img_src = np.ndarray(shape=(data.height, data.width, 3), dtype=np.uint8, buffer=data.data)
        self.img_c = cv2.cvtColor(img_src,cv2.COLOR_RGB2BGR)
        img0 = self.img_c.copy()
        # Padded resize  将图像填充w=h，并resize
        img, _, _, _ = letterbox(img0, new_shape=img_size)
        # Normalize RGB
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3*416*416
        img = np.ascontiguousarray(img, dtype=np.float32)  # uint8 to float32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        # Get detections
        img = torch.from_numpy(img).unsqueeze(0).to(device)
        pred, _ = model(img)
        # print(pred)
        det = non_max_suppression(pred, conf_thres, nms_thres)[0]
        # print(len(det))
        self.ang = 0.0
        self.x = v0
        self.y = u0
        if det is not None and len(det) > 0:
            # Rescale boxes from 416 to true image size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], img0.shape).round()
            # det[:, 4] = math.log(det[:, 4]/(1-det[:, 4]))
            # det[:, 5] = 0.5 * math.log((1 + det[:, 5]) / (1 - det[:, 5]))
            det[:, 4] = 0.5 * torch.atan2(det[:, 5], det[:, 4])
            det[:, 5] = det[:, 4] * 180 / math.pi
            *xyxy, ang, deg, conf, cls_conf, cls = det[0]
            c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
            print(c1,c2)
            center = [(c1[0]+c2[0])//2, (c1[1]+c2[1])//2]
            # bb = [center[0]-L2, center[1]-L2, center[0]+L2, center[1]+L2]
            self.ang = deg
            self.x = center[0]
            self.y = center[1]
            # print(center[0], center[1], deg)
            # Draw bounding boxes and labels of detections
            for *xyxy, ang, deg, conf, cls_conf, cls in det:
                # Add bbox to the image
                label = '%s %.2f %.1f' % (classes[int(cls)], conf, deg)
                plot_one_box(xyxy, img0, label=label, color=colors[int(cls)])

        # cv2.imshow("Current image", img_src)
        cv2.imshow("image0", img0)
        cv2.imshow("Target image", self.img_t)
        cv2.waitKey(3)
        detect_flag = True
        # self.test_detect()
        end = time.time()
        print('total time: {:.5}s'.format(end-start))

    def detectCallback(self, req):
        print('got data')
        res = yolo_graspResponse()
        res.pred = [self.x, self.y, self.ang]
        # 反馈数据
        return res

if __name__ == "__main__":

    img_proc = Image_Processor()
    try:
        rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
