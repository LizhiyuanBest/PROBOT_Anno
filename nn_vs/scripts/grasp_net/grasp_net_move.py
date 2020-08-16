#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy
import sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
import copy
import os
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nn_vs.srv import Image_data, Image_dataRequest
from nn_vs.msg import Image_Msg
import copy
import math
import time
from os.path import join
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

pi = 3.14159

# dx = dy = float(400) / 290  # mm/像素  1.37931
dx = float(150)/(497-113)
dy = float(150)/(418-306)

u0 = 240  
v0 = 320  

X0 = 0.3
Y0 = -0.045
Z = 0.085 + 0.25

Images_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images/image_200.jpg'
Detect_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/detect'

global detect_flag
detect_flag = False
Yaw = 0.0

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
        self.detect_request = Image_dataRequest()
        
        self.img_t = cv2.imread(Images_PATH)
        self.img_c = None

        # # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)

        self.image_sub = rospy.Subscriber("/probot_anno/hand_camera/hand_image_raw",Image,self.image_sub_callback)

        rospy.wait_for_service('/detect/server')
        self.detect_client = rospy.ServiceProxy('/detect/server', Image_data)
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
        global Yaw
        global detect_flag
        start = time.time()
        # cv2.imwrite(join(Detect_PATH, 'img1.jpg'), self.img_c)

        self.detect_request = Image_dataRequest()
        size = self.img_c.shape
        image1, image2 = Image_Msg(), Image_Msg()
        image1.height = size[0] # 480
        image1.width = size[1] # 640
        image1.channels = size[2] # 3
        image1.data = self.bridge.cv2_to_imgmsg(self.img_c, "bgr8").data # image_data
        self.detect_request.img1 = image1
        res = self.detect_client.call(self.detect_request)
        rospy.sleep(0.1)
        end = time.time()
        print('total time: {:.5f}s'.format(end - start))

        print('pred',res.pred)
        current_pose = self.arm.get_current_pose ()
        # print("current_pose",current_pose)

        rpy = self.arm.get_current_rpy()
        print(rpy[0]/pi*180,rpy[1]/pi*180,rpy[2]/pi*180)
        ang = rpy[2]/pi*180

        v1 = res.pred[0] - v0
        u1 = res.pred[1] - u0
        fai = res.pred[2]

        u1_ = v1*math.sin(rpy[2]) + u1*math.cos(rpy[2])
        v1_ = v1*math.cos(rpy[2]) - u1*math.sin(rpy[2])

        ey = -v1_ * dx
        ex = -u1_ * dy
        
        print("yaw={}, ex={}, ey={}".format(Yaw/pi*180,ex,ey))
        if math.fabs(ex) < 10.0 and math.fabs(ey) < 10.0 :
            x3 = current_pose.pose.position.x
            y3 = current_pose.pose.position.y
            ang += fai
            # Yaw += fai*pi/180
        else:
            x3 = current_pose.pose.position.x + ex/1000
            y3 = current_pose.pose.position.y + ey/1000
            
        
        self.arm.set_pose_target([x3, y3, Z, -180*pi/180, 0*pi/180, ang*pi/180])
        self.arm.go()
        # current_pose = self.arm.get_current_pose ()
        # print("current_pose",current_pose)
        # rospy.sleep(1)
        detect_flag = False
    

    def test_move(self):
        """ Testing move """
        rpy = self.arm.get_current_rpy()
        # print(rpy)

        self.arm.set_pose_target([X0+0.01, Y0+0.05, Z, -180*pi/180, 0*pi/180, 0*pi/180])
        self.arm.go()
        rospy.sleep(0.5)
        
        # self.arm.set_pose_target(target_pose)
        # self.arm.go() # 机械臂运动
        # rospy.sleep(1)
        # current_pose = self.arm.get_current_pose ()
        # print(current_pose)
        rpy = self.arm.get_current_rpy()
        print(rpy[0]/pi*180,rpy[1]/pi*180,rpy[2]/pi*180)

        # self.arm.set_pose_target([X0+0.01, Y0+0.1, Z, -180*pi/180, 0*pi/180, 0*pi/180])
        # self.arm.go()
        # rospy.sleep(0.5)
        # ang = 0
        # while True:
        #     rpy = self.arm.get_current_rpy()
        #     print(rpy[0]/pi*180,rpy[1]/pi*180,rpy[2]/pi*180)
        #     # a = rpy[2]/pi*180
        #     # if ang < -180: ang += 360
        #     # if ang > 180: ang -+ 360

        #     self.arm.set_pose_target([X0+0.01, Y0, Z, -180*pi/180, 0*pi/180, ang*pi/180])
        #     self.arm.go()
        #     rospy.sleep(0.5)
        #     ang += 10
        #     print(ang)
        #     rospy.sleep(1)



        # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)
        
       

if __name__ == "__main__":

    rb = Robot()
    global detect_flag
    try:
        rb.test_move()
        rospy.sleep(1)
        while True:
            if detect_flag == True:
                rospy.sleep(0.5)
                rb.test_detect()
            rospy.sleep(0.5)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
