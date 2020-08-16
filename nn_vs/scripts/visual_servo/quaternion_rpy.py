#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
import copy
import os
import random
import tf
import math

pi = 3.14159

X0 = 0.3
Y0 = -0.045
# 期望抓取位置
X = [X0, X0-0.025, X0-0.025, X0+0.025, X0+0.025]
Y = [Y0, Y0-0.025, Y0+0.025, Y0+0.025, Y0-0.025]
Z = 0.085 + 0.15

# 初始化move_group的API
moveit_commander.roscpp_initialize(sys.argv)
# 初始化ROS节点
rospy.init_node('quaternion', anonymous=True)
# 初始化需要使用move group控制的机械臂中的arm group
arm = moveit_commander.MoveGroupCommander('manipulator')
# 设置机械臂运动的允许误差值
arm.set_goal_joint_tolerance(0.002)
# 设置允许的最大速度和加速度
arm.set_max_acceleration_scaling_factor(1)
arm.set_max_velocity_scaling_factor(1)
# base
base_frame_id = "/base_footprint"
end_effector_link = arm.get_end_effector_link()

# 控制机械臂先回到初始化位置
# arm.set_named_target('home')
# arm.go()
# rospy.sleep(1)

arm.set_pose_target([X[0], Y[0], Z, (-180)*pi/180, (30)*pi/180, 60*pi/180])
arm.go()
rospy.sleep(0.2)
# read current pose from simulation
current_pose = arm.get_current_pose()
print(current_pose)
rpy = arm.get_current_rpy()
print(rpy)

q = [current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, current_pose.pose.orientation.w]
r, p, y = tf.transformations.euler_from_quaternion(q)
print(r, p, y)

x,y,z,w = q
print(x,y,z,w)
rpy_r = math.atan2(2.0*(x*w+y*z), 1.0-2.0*(x*x+y*y))
rpy_p = math.asin(2.0*(w*y - z*x))
rpy_y = math.atan2(2.0*(z*w+y*x), 1.0-2.0*(z*z+y*y))

print(rpy_r, rpy_p, rpy_y)
