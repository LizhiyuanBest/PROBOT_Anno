#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
import copy
# import Quaternion

pi = 3.14159

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
        self.arm.set_max_acceleration_scaling_factor(0.8)
        self.arm.set_max_velocity_scaling_factor(0.8)
        # base
        self.base_frame_id = "/base_footprint"
        self.end_effector_link = self.arm.get_end_effector_link()
        print(self.end_effector_link)
        
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        rospy.sleep(1)
        
    def pose_transform(self, curr_pose, delte_pose):
        """ pose transform  return pose """
        target_pose = copy.deepcopy(curr_pose)
        # target_pose.pose.position.x = 0.235270741065
        # target_pose.pose.position.y = 0.097024760869
        # target_pose.pose.position.z = 0.203904230734

        

        target_pose.pose.orientation.x = -0.999513345318
        target_pose.pose.orientation.y = 0.0311825508069
        target_pose.pose.orientation.z = -0.000829331830071
        target_pose.pose.orientation.w = 0.000182385374545
        target_pose.header.stamp = rospy.Time.now() # update time in ROS
        return target_pose

    def test_move(self):
        """ Testing move """
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame_id #  must have this, pose is based on "base_footprint"
        target_pose.header.stamp = rospy.Time.now() # update time in ROS
        target_pose.pose.position.x = 0.235270741065
        target_pose.pose.position.y = 0.097024760869
        target_pose.pose.position.z = 0.203904230734
        target_pose.pose.orientation.x = -0.999513345318
        target_pose.pose.orientation.y = 0.0311825508069
        target_pose.pose.orientation.z = -0.000829331830071
        target_pose.pose.orientation.w = 0.000182385374545

        rpy = self.arm.get_current_rpy()
        print(rpy)
        # self.arm.set_position_target([0.235, 0.1, 0.2],end_effector_link=self.end_effector_link)
        # self.arm.go()
        # rospy.sleep(1)
        # self.arm.set_rpy_target([-180/180*pi, 0/180*pi, 0/180*pi],end_effector_link=self.end_effector_link)
        # self.arm.go()
        # rospy.sleep(1)
        self.arm.set_pose_target([0.235, 0.1, 0.2, -180*pi/180, 0*pi/180, 0*pi/180])
        self.arm.go()
        rospy.sleep(1)
        
        # self.arm.set_pose_target(target_pose)
        # self.arm.go() # 机械臂运动
        rospy.sleep(1)
        current_pose = self.arm.get_current_pose ()
        print(current_pose)
        rpy = self.arm.get_current_rpy()
        print(rpy)

        # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [0.391410, -0.676384, -0.376217, 0.0, 1.052834, 0.454125]
        # self.arm.set_joint_value_target(joint_positions)
        
        # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)
        
       

if __name__ == "__main__":

    # q1 = [-0.999513345318,0.0311825508069,-0.000829331830071,0.000182385374545]
    # q2 = [0,0,1,0]
    # q = Quaternion_multiply(q1, q2)
    # q = Quaternion_normal(q)
    # print(q)

    rb = Robot()

    try:
        rb.test_move()
    except rospy.ROSInterruptException:
        pass
