#!/usr/bin/env python
# -*- coding: utf-8 -*-


import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped
import copy
import os
from nn_vs.srv import Save_image, Save_imageRequest
import random

pi = 3.14159

X0 = 0.3
Y0 = -0.045
# 期望抓取位置
X = [X0, X0-0.025, X0-0.025, X0+0.025, X0+0.025]
Y = [Y0, Y0-0.025, Y0+0.025, Y0+0.025, Y0-0.025]
Z = 0.085 + 0.25

roll = [x for x in range(-5,6,1)]
pitch = [x for x in range(-5,6,1)]
yaw = [x for x in range(-10,11,1)]

pz = [float(x)/1000 for x in range(0,11,1)]
px = [float(x)/1000 for x in range(-5,6,1)]
py = [float(x)/1000 for x in range(-5,6,1)]
pxy = [[-0.005, 0], [0.005, 0], [-0.0025, 0], [0.0025, 0], [0, 0], \
        [0, 0.005], [0, -0.005], [0, 0.0025], [0, -0.0025], \
        [0.0025, 0.0025], [-0.0025, 0.0025], [0.0025, -0.0025], [-0.0025, -0.0025]]

Yaw = [0, 30, 60, 90] # Yaw = [0, 30, 60, 90]

ROOT_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/poses'

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
        self.arm.set_goal_joint_tolerance(0.0002)
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(0.8)
        self.arm.set_max_velocity_scaling_factor(0.8)
        # base
        self.base_frame_id = "/base_footprint"
        self.end_effector_link = self.arm.get_end_effector_link()
        # print(self.end_effector_link)
        self.id = 1 # 1, 937, 1873, 2809, 3745, 
        self.save_request = Save_imageRequest()
        
        # 控制机械臂先回到初始化位置
        # self.arm.set_named_target('home')
        # self.arm.go()
        # rospy.sleep(1)

        rospy.wait_for_service('/image_save')
        self.image_save_client = rospy.ServiceProxy('/image_save', Save_image)
        rospy.sleep(0.2)
        
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

    def write_pose(self, file_name, current_pose, rpy):
        """ write current pose and rpy in file """
        # write the pose in file
        file_handle = open(os.path.join(ROOT_PATH, file_name),mode='w+')
        file_handle.write('# position\n')
        s = str(current_pose.pose.position.x)
        file_handle.write('x='+s+'\n')
        s = str(current_pose.pose.position.y)
        file_handle.write('y='+s+'\n')
        s = str(current_pose.pose.position.z)
        file_handle.write('z='+s+'\n')
        file_handle.write('# quaternion\n')
        s = str(current_pose.pose.orientation.x)
        file_handle.write('x='+s+'\n')
        s = str(current_pose.pose.orientation.y)
        file_handle.write('y='+s+'\n')
        s = str(current_pose.pose.orientation.z)
        file_handle.write('z='+s+'\n')
        s = str(current_pose.pose.orientation.w)
        file_handle.write('w='+s+'\n')
        file_handle.write('# rpy\n')
        s = str(rpy[0])
        file_handle.write('r='+s+'\n')
        s = str(rpy[1])
        file_handle.write('p='+s+'\n')
        s = str(rpy[2])
        file_handle.write('y='+s+'\n')

        file_handle.close()

    def auto_simple(self):
        # vertical cylinder with radius 5mm and height 10 mm
        # sampled uniformly from -5 degrees to 5 degrees for roll and pitch 
        # and -10 degrees to 10 degrees for yaw.
        # 正方形四个顶点加中心点 共五个点， 每个点旋转0°，30°，60°，90°， 每次均匀采样200个数据
        # 5*4*200 = 4000
        rpys = []
        for r in roll:
            for p in pitch:
                for y in yaw:
                    rpys.append([r,p,y])
        random.shuffle(rpys)
        # pos = []
        # for z in pz:
        #     for x in px:
        #         for y in py:
        #             pos.append([x,y,z])
        # print(len(rpys), len(pos))
        n = len(rpys)
        idx = 0
        for YAW in Yaw:
            for z in pz:
                for x in px:
                    for y in py:
                        # 遍历位置， 姿态随机选3个
                        for _ in range(3):
                            r = rpys[idx][0]
                            p = rpys[idx][1]
                            ya = rpys[idx][2]
                            idx += 1
                            if idx == n: idx = 0
                            print(self.id,x,y,z,r,p,ya)
                            # move to the target position
                            self.arm.set_pose_target([X[0]+x, Y[0]+y, Z+z, (-180+r)*pi/180, (0+p)*pi/180, (YAW+ya)*pi/180])
                            self.arm.go()
                            rospy.sleep(0.2)
                            # read current pose from simulation
                            current_pose = self.arm.get_current_pose()
                            # print(current_pose)
                            rpy = self.arm.get_current_rpy()
                            # print(rpy)
                            # write the pose in file
                            self.write_pose(str(self.id)+'.txt', current_pose, rpy)
                            self.save_request.file_name = str(self.id)+'.jpg'
                            self.image_save_client.call(self.save_request)
                            self.id += 1

    def auto_simple1(self):
        # vertical cylinder with radius 5mm and height 10 mm
        # sampled uniformly from -5 degrees to 5 degrees for roll and pitch 
        # and -10 degrees to 10 degrees for yaw.
        # 正方形四个顶点加中心点 共五个点， 每个点旋转0°，30°，60°，90°， 每次均匀采样200个数据
        # 5*4*200 = 4000
        for YAW in Yaw:
            for z in pz:
                for x,y in pxy:
                    # 遍历位置， 姿态随机选3个
                    for _ in range(3):
                        r = random.choice(roll)
                        p = random.choice(pitch)
                        ya = random.choice(yaw)
                        print(self.id,x,y,z,r,p,ya)
                        # move to the target position
                        self.arm.set_pose_target([X[0]+x, Y[0]+y, Z+z, (-180+r)*pi/180, (0+p)*pi/180, (YAW+ya)*pi/180])
                        self.arm.go()
                        rospy.sleep(0.2)
                        # read current pose from simulation
                        current_pose = self.arm.get_current_pose()
                        # print(current_pose)
                        rpy = self.arm.get_current_rpy()
                        # print(rpy)
                        # write the pose in file
                        self.write_pose(str(self.id)+'.txt', current_pose, rpy)
                        self.save_request.file_name = str(self.id)+'.jpg'
                        self.image_save_client.call(self.save_request)
                        self.id += 1 

    def test_move(self):
        """ Testing move """
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = self.base_frame_id #  must have this, pose is based on "base_footprint"
        # target_pose.header.stamp = rospy.Time.now() # update time in ROS
        # target_pose.pose.position.x = 0.235270741065
        # target_pose.pose.position.y = 0.097024760869
        # target_pose.pose.position.z = 0.203904230734
        # target_pose.pose.orientation.x = 0.999513345318
        # target_pose.pose.orientation.y = -0.0311825508069
        # target_pose.pose.orientation.z = 0.000829331830071
        # target_pose.pose.orientation.w = -0.000182385374545
        # self.arm.set_pose_target(target_pose)
        # self.arm.go()
        # rospy.sleep(1)

        # rpy = self.arm.get_current_rpy()
        # print(rpy)

        self.arm.set_pose_target([X[0], Y[0], Z, -180*pi/180, 0*pi/180, 0*pi/180])
        self.arm.go()
        rospy.sleep(0.5)
        
        # # self.arm.set_pose_target(target_pose)
        # # self.arm.go() # 机械臂运动
        # # rospy.sleep(1)
        # current_pose = self.arm.get_current_pose ()
        # print(current_pose)
        # rpy = self.arm.get_current_rpy()
        # print(rpy)

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
        # rb.auto_simple()
    except rospy.ROSInterruptException:
        pass
