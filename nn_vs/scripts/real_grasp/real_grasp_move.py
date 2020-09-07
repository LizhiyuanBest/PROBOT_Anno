#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import copy
import os
from nn_vs.srv import yolo_grasp, yolo_graspRequest
import math
import time

pi = 3.14159

# dx = dy = float(400) / 290  # mm/像素  1.37931
dx = float(300)/(560-176)
dy = float(300)/(418-35)

u0 = 240  
v0 = 320  

X0 = 0.3
Y0 = -0.045
Z = 0.085 + 0.25

Z0 = 350 # mm
ZG = 160 # mm

class Robot:

    def __init__(self):
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
        self.detect_request = yolo_graspRequest()
        
        # # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)
        self.test_move()

        rospy.wait_for_service('/detect/server')
        self.detect_client = rospy.ServiceProxy('/detect/server', yolo_grasp)
        rospy.sleep(0.2)
    

    def test_detect(self):
        start = time.time()
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

        u1_ = v1*math.sin(-rpy[2]) + u1*math.cos(-rpy[2])
        v1_ = v1*math.cos(-rpy[2]) - u1*math.sin(-rpy[2])

        ey = -v1_ * dx
        ex = -u1_ * dy
        
        print("yaw={}, ex={}, ey={}".format(ang,ex,ey))
        if math.fabs(ex) > 5.0 or math.fabs(ey) > 5.0 or math.fabs(fai) > 2.0:
            x3 = current_pose.pose.position.x + ex/1000
            y3 = current_pose.pose.position.y + ey/1000
            if ang + fai > 180: fai -= 180
            if ang + fai < -180: fai += 180
            ang += fai
        else:
            x3 = current_pose.pose.position.x
            y3 = current_pose.pose.position.y
        # if math.fabs(ex) < 10.0 and math.fabs(ey) < 10.0 :
        #     x3 = current_pose.pose.position.x
        #     y3 = current_pose.pose.position.y
        #     if ang + fai > 180: fai -= 180
        #     if ang + fai < -180: fai += 180
        #     ang += fai
        #     # Yaw += fai*pi/180
        # else:
        #     x3 = current_pose.pose.position.x + ex/1000
        #     y3 = current_pose.pose.position.y + ey/1000
            
        
        self.arm.set_pose_target([x3, y3, Z, -180*pi/180, 0*pi/180, ang*pi/180])
        self.arm.go()
        # current_pose = self.arm.get_current_pose ()
        # print("current_pose",current_pose)
        # rospy.sleep(1)
    

    def test_move(self):
        """ Testing move """
        rpy = self.arm.get_current_rpy()
        # print(rpy)

        # for i in range(10000):
        #     rpy = self.arm.get_current_rpy()
        #     print(rpy)
        #     ang = rpy[2]/pi*180
        #     fai = 10 
        #     if ang + fai > 180: fai -= 180
        #     if ang + fai < -180: fai += 180
        #     self.arm.set_pose_target([X0+0.01, Y0+0.05, Z, -180*pi/180, 0*pi/180, (ang + fai)*pi/180])
        #     self.arm.go()
        #     rospy.sleep(0.5)


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

       

if __name__ == "__main__":

    rb = Robot()
    try:
        # rb.test_move()
        rospy.sleep(1)
        while True:
            rospy.sleep(0.1)
            rb.test_detect()
            rospy.sleep(0.1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
