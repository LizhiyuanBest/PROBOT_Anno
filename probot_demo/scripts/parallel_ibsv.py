#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import copy
import math
from probot_demo.srv import Img_process, Img_processRequest
from probot_demo.msg import Process_Result
import moveit_commander

import tf
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point
from std_msgs.msg import Float32MultiArray, String, Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math
import numpy as np

usleep = lambda x: time.sleep(x/1000000.0)  # python 当中没有usleep

dx = dy = float(400) / 290  # mm/像素  1.37931
pi = 3.141592

u0 = 400
v0 = 400

P = 0.001
D = 0

L1 = 284
L2 = 225
L3 = 228.9
L4 = 55

limit = [[-pi, pi], [-2.0071, 2.0071], [-0.6981, 3.8397], [-pi, pi], [-0.7854, 3.9270], [-pi, pi]]

s_star = [[400.0,400.0], [448.0,400.0], [400,352]]

K1 = 0.005
K2 = 0.01
K3 = 0.5

K = K1

class Robot:

    def __init__(self, pregrasp):
        # ROS节点初始化
        rospy.init_node('parallel_ibvs', anonymous=True)
        self.grasp_color = ['green']  # ['green', 'red', 'blue']
        self.joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.cnt = 0
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        self.end_link = self.arm.get_end_effector_link()
        # rospy.loginfo(self.end_link)

        self.vel_pub = rospy.Publisher("/probot_anno/arm_vel_controller/command", Float64MultiArray, queue_size=10)
        self.joint_sub = rospy.Subscriber("/probot_anno/joint_states", JointState , self.joints_callback)
        self.center_sub = rospy.Subscriber("/hand_camera/region", Float32MultiArray, self.hand_camera_callback)

        rospy.sleep(3)

    def get_e_L(self, data):
        L = np.zeros((6,6), dtype=np.float)
        e = np.zeros((6,1), dtype=np.float)
        z = 0.1
        s_d = list(np.array(s_star).flat)
        for i in range(0, 5, 2): 
            e[i+1] = s_d[i] - data[i]
            e[i] = s_d[i+1] - data[i+1]
            x = data[i+1]  
            y = data[i]
            L[i] = [-1/z, 0, x/z, x*y, -(1+x*x), y]
            L[i+1] = [0, -1/z, y/z, (1+y*y), -x*y, -x]
        return e, L

    def hand_camera_callback(self, msg):
        """ get center of object """
        rospy.loginfo(msg.data)
        self.cnt += 1
        print('self.cnt', self.cnt)
        area = math.fabs((msg.data[0]-msg.data[2]) * (msg.data[1]-msg.data[5]))
        print('area',area)
        ez = -(4000 - area) * 0.0001
        # a
        # center = [msg.data[0], msg.data[1]]
        # ex = (400 - center[1]) * P
        # ey = (400 - center[0]) * P
        # # print("center={} ex={} ey={}".format(center,ex,ey))
        # speedl = np.matrix([ex, ey, ez, 0.0, 0.0, 0.0])
        # print('speedl_1', speedl)
        # b
        e, L = self.get_e_L(msg.data)
        # print(L)
        print(e.T)
        L_pinv = np.linalg.pinv(L)
        # print(L_pinv)
        global K 
        vc = -K * np.dot(L_pinv, e)
        speedl = np.matrix(list(np.array(vc).flat))
        print('speedl_2',speedl)

        # print(self.arm.get_current_pose())
        # rospy.loginfo(self.joints)
        # current_pose = self.arm.get_current_pose(self.end_link)
        # print(current_pose)

        jacob = self.arm.get_jacobian_matrix(self.joints) 
        # print(jacob)
        inv_jacob = np.linalg.inv(np.matrix(jacob))
        # print(inv_jacob)
        # speedl = np.matrix([ex, ey, 0.0, 0.0, 0.0, 0.0])
        speedj = inv_jacob * speedl.T
        speedj = list(np.array(speedj).flat)
        print('speedj',speedj)
        flag_limit = self.judge_limit(speedj, 0.1)
        # rospy.loginfo(flag_limit)
        if flag_limit and self.cnt<300:
            vel_msg = Float64MultiArray(data=speedj)
        else:
            vel_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        if np.max(e) < 30:
            K = K2
        elif np.max(e) < 10:
            K = K3
        elif np.max(e) < 3:
            vel_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self.vel_pub.publish(vel_msg)

    def judge_limit(self, speedj, d_t):
        """ judge the joints limit """
        for i in range(6):
            if limit[i][0] < self.joints[i] + speedj[i]*d_t < limit[i][1]:
                continue
            return False
        return True

    def test_jacob(self):
        # jacob = self.arm.get_jacobian_matrix(self.joints)
        # print(jacob)
        # inv_jacob = np.linalg.inv(np.matrix(jacob))
        # print(inv_jacob)

        del_t = 0.01#//加速的时间间隔delta a，单位：s
        n = 0 #/第几个
        speedl = [0, 0, 0, 0, 0, 0]#       //末端线速度    zyx
        speedj = [0, 0, 0, 0, 0, 0]#       //6关节角速度
        accel1 = [0.003536, 0.003536, 0, 0, 0, 0]#     // 加速速度矢量 标量为0.005m/s^2
        accel2 = [-0.002829, -0.002829, 0, 0, 0, 0]#     // 减速度矢量 标量为0.004m/s^2   这两个加速度都是末端的笛卡尔加速度

        targetPose = [0, 0, 0, 0.0, 0, 0]#  //运动的起始位置

        # //加速阶段，4s，加速度为accel1
        for n in range(400): #speed up:4s

            # //求雅克比矩阵，关节角速度与末端位姿速度之间的雅可比矩阵，位姿速度=雅可比矩阵×角速度
            # //雅克比矩阵与机械臂当前的位置状态有关，所以每个步长都要计算一次
            jacob = self.arm.get_jacobian_matrix(self.joints)
            print(jacob)
            inv_jacob = np.linalg.inv(np.matrix(jacob))
            print(inv_jacob)
            # //更新速度
            speedl = np.matrix(speedl) + np.matrix(accel1) * del_t

            # print('l',speedl)
            speedl = np.array(speedl.T).flat
            # print('l',speedl)
            speedj = inv_jacob * speedl #        //得到弧度制
            # print('j',speedj)
            speedj = list(np.array(speedj).flat)
            vel_msg = Float64MultiArray(data=speedj)
            # print('vel_msg',vel_msg)
            self.vel_pub.publish(vel_msg)
            usleep(9500) #     //9500微秒，9.5毫秒
            # //然后修改关节角度
            # joint_group_positions[0] = joint_group_positions[0] + speedj[0] * 0.01;
            # joint_group_positions[1] = joint_group_positions[1] + speedj[1] * 0.01;
            # joint_group_positions[2] = joint_group_positions[2] + speedj[2] * 0.01;
            # joint_group_positions[3] = joint_group_positions[3] + speedj[3] * 0.01;
            # joint_group_positions[4] = joint_group_positions[4] + speedj[4] * 0.01;
            # joint_group_positions[5] = joint_group_positions[5] + speedj[5] * 0.01;

        #5s的减速阶段，减速速度为accel2
        for n in range (500):

            # //求雅克比矩阵
            jacob = self.arm.get_jacobian_matrix(self.joints)
            print(jacob)
            inv_jacob = np.linalg.inv(np.matrix(jacob))
            print(inv_jacob)

            # //更新速度
            speedl = np.matrix(speedl) + np.matrix(accel2) * del_t

            # print('l',speedl)
            speedl = np.array(speedl.T).flat
            # print('l',speedl)
            speedj = inv_jacob * speedl #        //得到弧度制
            # print('j',speedj)
            speedj = list(np.array(speedj).flat)
            vel_msg = Float64MultiArray(data=speedj)
            # print('vel_msg',vel_msg)
            self.vel_pub.publish(vel_msg)
            usleep(9500) #     //9500微秒，9.5毫秒

            # //然后修改关节角度，更新关节角度
            # joint_group_positions[0] = joint_group_positions[0] + speedj[0] * 0.01;
            # joint_group_positions[1] = joint_group_positions[1] + speedj[1] * 0.01;
            # joint_group_positions[2] = joint_group_positions[2] + speedj[2] * 0.01;
            # joint_group_positions[3] = joint_group_positions[3] + speedj[3] * 0.01;
            # joint_group_positions[4] = joint_group_positions[4] + speedj[4] * 0.01;
            # joint_group_positions[5] = joint_group_positions[5] + speedj[5] * 0.01;

    def test_line(self):
        speedl = [0.00, -0.05, -0.0, 0, 0, 0]#       //末端线速度    zyx   
        for _ in range(15): #speed up:4s

            # //求雅克比矩阵，关节角速度与末端位姿速度之间的雅可比矩阵，位姿速度=雅可比矩阵×角速度
            # //雅克比矩阵与机械臂当前的位置状态有关，所以每个步长都要计算一次
            jacob = self.arm.get_jacobian_matrix(self.joints)
            # print(jacob)
            inv_jacob = np.linalg.inv(np.matrix(jacob))
            # print(inv_jacob)
            speedj = inv_jacob * np.matrix(speedl).T #        //得到弧度制
            # print('j',speedj)
            speedj = list(np.array(speedj).flat)
            vel_msg = Float64MultiArray(data=speedj)
            # print('vel_msg',vel_msg)
            self.vel_pub.publish(vel_msg)
            usleep(9500) #     //9500微秒，9.5毫秒
        vel_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # print('vel_msg',vel_msg)
        self.vel_pub.publish(vel_msg)

    def joints_callback(self, msg):
        """ get center of object """
        # rospy.loginfo(msg)
        # print(msg.position)
        self.joints = list(msg.position)



if __name__ == "__main__":
	
    # rospy.sleep(2.0)
    rb = Robot([0.3, 0.0, 0.2])  # 创建机器人实例
    # rospy.sleep(1.0)
    # while True:
    #     pub_msg = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    #     # pub_msg.data.append(0.1)
    # rb.test_jacob()
    #     rb.vel_pub.publish(pub_msg)
    # rb.test_line()
    try:
        rospy.spin()  # 循环等待回调函数
    except KeyboardInterrupt:
        print("Shutting down")
