#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
import copy
import os
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nn_vs.srv import Detect_data, Detect_dataRequest
from nn_vs.msg import Image_Msg 
import copy
import math
from os.path import join
import sys
import cv2
import numpy as np

pi = 3.14159

X0 = 0.3
Y0 = -0.045
# 期望抓取位置
X = [X0, X0-0.025, X0-0.025, X0+0.025, X0+0.025]
Y = [Y0, Y0-0.025, Y0+0.025, Y0+0.025, Y0-0.025]
Z = 0.085 + 0.15

global detect_flag
detect_flag = False

Poses_PATH = '/home/li/visual_servo/delta_pose/data/poses'
Images_PATH = '/home/li/visual_servo/delta_pose/data/images/val'
Weight_PATH = '/home/li/visual_servo/delta_pose/weights'
weights_file = 'best_models.pt'
Detect_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/detect'

def Quaternion_multiply(q1,q2):
    """
    q1 * q2 = 
    (w1*w2 - x1*x2 - y1*y2 - z1*z2)   + (w1*x2 + x1*w2 + y1*z2 - z1*y2) i +
    (w1*y2 - x1*z2 + y1*w2 + z1*x2) j + (w1*z2 + x1*y2 - y1*x2 + z1*w2) k
    """
    x1,y1,z1,w1 = q1
    x2,y2,z2,w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return [x,y,z,w]


def Quaternion_normal(q):
    x,y,z,w = q
    mod = pow((pow(x,2)+pow(y,2)+pow(z,2)+pow(w,2)), 1/2)
    return [x/mod, y/mod, z/mod, w/mod]


def Quaternion_to_rpy(q):
    x, y, z, w = q
    rpy_r = math.atan2(2.0 * (x * w + y * z), 1.0 - 2.0 * (x * x + y * y))
    rpy_p = math.asin(2.0 * (w * y - z * x))
    rpy_y = math.atan2(2.0 * (z * w + y * x), 1.0 - 2.0 * (z * z + y * y))
    # return [rpy_r, rpy_p, rpy_y]
    return [rpy_r/pi*180, rpy_p/pi*180, rpy_y/pi*180]


class Robot:

    def __init__(self):
        self.bridge = CvBridge()
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('test_move', anonymous=True)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        # 设置机械臂运动的允许误差值
        self.arm.set_goal_joint_tolerance(0.001)
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)
        # base
        self.base_frame_id = "/world"
        self.end_effector_link = self.arm.get_end_effector_link()
        # print(self.end_effector_link)
        self.id = 1 # 1, 937, 1873, 2809, 3745, 
        self.detect_request = Detect_dataRequest()
        
        image_list = os.listdir(Images_PATH)
        print(len(image_list))
        img2_file = image_list[123]
        self.img_t = cv2.imread(join(Images_PATH, img2_file))
        self.img_c = np.zeros((480, 640, 3), dtype=np.uint8) 

        # # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)

        self.image_sub = rospy.Subscriber("/probot_anno/hand_camera/hand_image_raw",Image,self.image_sub_callback)

        rospy.wait_for_service('/detect/server')
        self.detect_client = rospy.ServiceProxy('/detect/server', Detect_data)
        rospy.sleep(0.2)
    

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        global detect_flag
        try:
            self.img_c = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Current image", self.img_c)
            cv2.imshow("Target image", self.img_t)
            cv2.waitKey(3)
            detect_flag = True
            # self.test_detect()
        except CvBridgeError as e:
            print(e) 

    def test_detect(self):
        
        self.detect_request = Detect_dataRequest()
        size = self.img_c.shape
        image1, image2 = Image_Msg(), Image_Msg()
        image1.height = size[0] # 480
        image1.width = size[1] # 640
        image1.channels = size[2] # 3
        image1.data = self.bridge.cv2_to_imgmsg(self.img_c, "bgr8").data # image_data
        image2.height = size[0] # 480
        image2.width = size[1] # 640
        image2.channels = size[2] # 3
        image2.data = self.bridge.cv2_to_imgmsg(self.img_t, "bgr8").data # image_data
        self.detect_request.img1 = image1
        self.detect_request.img2 = image2
        res = self.detect_client.call(self.detect_request)
        rospy.sleep(0.1)
        print('res',res)
        current_pose = self.arm.get_current_pose ()
        print("current_pose",current_pose)
        target_pose = copy.copy(current_pose)
        target_pose.header.stamp = rospy.Time.now() # update time in ROS
        x2 = target_pose.pose.position.x + res.pred[0]
        y2 = target_pose.pose.position.y + res.pred[1]
        z2 = target_pose.pose.position.z + res.pred[2]
        target_pose.pose.position.x += res.pred[0]
        target_pose.pose.position.y += res.pred[1]
        target_pose.pose.position.z += res.pred[2]
        q1 = [target_pose.pose.orientation.x, target_pose.pose.orientation.y,
            target_pose.pose.orientation.z, target_pose.pose.orientation.w]
        q2 = Quaternion_multiply(res.pred[3:], q1)
        q2 = Quaternion_normal(q2)
        rpy2 = Quaternion_to_rpy(q2)
        print('target',[x2,y2,z2],rpy2)
        # target_pose.pose.orientation.x = -q2[0]
        # target_pose.pose.orientation.y = -q2[1]
        # target_pose.pose.orientation.z = -q2[2]
        # target_pose.pose.orientation.w = -q2[3]
        # print('target_pose', target_pose)
        # self.arm.set_pose_target(target_pose)
        self.arm.set_pose_target([x2, y2, z2, rpy2[0]*pi/180, rpy2[1]*pi/180, rpy2[2]*pi/180])
        self.arm.go()
        current_pose = self.arm.get_current_pose ()
        print("current_pose",current_pose)
        # rospy.sleep(1)
    

    def test_move(self):
        """ Testing move """
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = self.base_frame_id #  must have this, pose is based on "base_footprint"
        # target_pose.header.stamp = rospy.Time.now() # update time in ROS
        # target_pose.pose.position.x = 0.235270741065
        # target_pose.pose.position.y = 0.097024760869
        # target_pose.pose.position.z = 0.203904230734
        # target_pose.pose.orientation.x = -0.999513345318
        # target_pose.pose.orientation.y = 0.0311825508069
        # target_pose.pose.orientation.z = -0.000829331830071
        # target_pose.pose.orientation.w = 0.000182385374545

        rpy = self.arm.get_current_rpy()
        # print(rpy)

        self.arm.set_pose_target([X[0]+0.01, Y[0], Z, -180*pi/180, 0*pi/180, 0*pi/180])
        self.arm.go()
        rospy.sleep(0.5)
        
        # self.arm.set_pose_target(target_pose)
        # self.arm.go() # 机械臂运动
        # rospy.sleep(1)
        current_pose = self.arm.get_current_pose ()
        # print(current_pose)
        rpy = self.arm.get_current_rpy()
        print(rpy[0]/pi*180,rpy[1]/pi*180,rpy[2]/pi*180)

        # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        # self.arm.set_joint_value_target(joint_positions)
        
        # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)
        
       

if __name__ == "__main__":

    rb = Robot()
    global detect_flag
    try:
        rb.test_move()
        rospy.sleep(2)
        # rb.test_detect()
        while True:
            if detect_flag == True:
                rospy.sleep(1)
                rb.test_detect()
            rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
