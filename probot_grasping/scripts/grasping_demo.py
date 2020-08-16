#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:  # ros 路径会对导入cv库造成影响
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import random
import copy
import roslib
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, PointStamped, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from vision_manager import VisionManager
import moveit_commander
from moveit_commander import RobotCommander, MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from std_msgs.msg import String, Float64



class GraspingDemo:

    def __init__(self, pregrasp_x, pregrasp_y, pregrasp_z, length=1., breadth=0.6):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.armgroup = moveit_commander.MoveGroupCommander('manipulator')
        # 获取终端link的名称
        self.end_effector_link = self.armgroup.get_end_effector_link()
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        self.armgroup.set_pose_reference_frame(reference_frame)
        self.grippergroup = moveit_commander.MoveGroupCommander('gripper')
        self.grippergroup.set_named_target("open")
        self.vMng_ = VisionManager(length, breadth)

        self.pregrash_x = pregrasp_x
        self.pregrash_y = pregrasp_y
        self.pregrash_z = pregrasp_z
        
        self.tf_camera_to_robot = tf.TransformListener()
        got_tf = False
        while not got_tf:
            try: # 如果查询得到的话，就将结果保存到camera_to_robot_，保存x,y,z和四元数一共7个值
                (self.trans,self.rot) = self.tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", rospy.Time(0))
                rospy.loginfo("[adventure_tf]: got")
                # print("trans ",self.trans)
                # print("rot ",self.rot)
                got_tf = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.loginfo("[adventure_tf]: (wait)")
                rospy.sleep(1.0)
            
        self.grasp_running = False

        
        rospy.loginfo("Getting into the Grasping Position....")
        # 调用该函数控制机械臂运动到设定的位置
        self.attainPosition(pregrasp_x, pregrasp_y, pregrasp_z)
        rospy.sleep(2.0)
        # Subscribe to input video feed and publish object location
        self.image_sub_ = rospy.Subscriber("/probot_anno/camera/image_raw", Image, self.imageCb)


    def attainPosition(self, x, y, z):
        # rospy.loginfo("The attain position function called")

        # 获取当前位置
        curr_pose = self.armgroup.get_current_pose(self.end_effector_link)
        #print("curr_pose", curr_pose)
        target_pose1 = copy.copy(curr_pose)
        # 设置抓取前的机械臂位置
        target_pose1.pose.position.x = x
        target_pose1.pose.position.y = y
        target_pose1.pose.position.z = z
        #print("target_pose1", target_pose1)

        self.armgroup.set_pose_target(target_pose1, self.end_effector_link)
        self.armgroup.go() # 机械臂运动


    def imageCb(self, msg):
        # rospy.loginfo("The image callback function called")
        rospy.sleep(0.1)
        if not self.grasp_running:
            rospy.loginfo("Processing the Image to locate the Object...")
            try: # 将图像变换到opencv当中
                cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
                # cv2.imshow("Image window", cv_image)
                # cv2.waitKey(300)
            except CvBridgeError as e:
                rospy.loginfo("cv_bridge exception: ", e.what())

            rospy.loginfo("Image Message Received")
            # 调用vision_manager中的函数获取目标的位置，位置坐标是以摄像头中心点的位置作为0坐标
            obj_x, obj_y = self.vMng_.get2DLocation(cv_image)
            print(" X-Co-ordinate in Camera Frame : ", obj_x)
            print(" Y-Co-ordinate in Camera Frame : ", obj_y)

            # 通过坐标变换，将二维坐标变换为相机坐标系下的三维坐标，在本程序中与URDF建模有关系
            obj_camera_frame  = PointStamped(header=rospy.Header(stamp = rospy.get_rostime(),frame_id="/camera_link"), point=Point(x=0.45,y=-obj_x,z=-obj_y))
            
            # 关键的一行代码，将相机坐标系下的位置转化为base_link坐标系下的坐标
            self.obj_robot_frame = self.tf_camera_to_robot.transformPoint(target_frame="/base_link", ps=obj_camera_frame)
            print(" X-Co-ordinate in Robot Frame : ", self.obj_robot_frame.point.x)
            print(" Y-Co-ordinate in Robot Frame : ", self.obj_robot_frame.point.y)
            print(" Z-Co-ordinate in Robot Frame : ", self.obj_robot_frame.point.z)
            # self.obj_robot_frame = Vector3

            self.grasp_running = True


    def attainObject(self):
        # rospy.loginfo("The attain Object function called")
        self.attainPosition(self.obj_robot_frame.point.x, self.obj_robot_frame.point.y+0.057, self.obj_robot_frame.point.z + 0.25)

        # self.attainPosition(0.30, -0.0029, 0.50)
        # Open Gripper
        rospy.sleep(1.0)
        self.grippergroup.set_named_target("open")
        self.grippergroup.go()

        # Slide down the Object
        curr_pose = self.armgroup.get_current_pose(self.end_effector_link)
        target_pose1 = Pose(curr_pose.pose.position, curr_pose.pose.orientation)
        print(target_pose1)

        target_pose1.position.z = 0.4
        self.armgroup.set_pose_target(target_pose1)
        self.armgroup.go()


    def grasp(self):
        # rospy.loginfo("The Grasping function called")
        # rospy.sleep(1.0)
        self.grippergroup.set_named_target("close")
        self.grippergroup.go()


    def lift(self):
        # rospy.loginfo("The lift function called")
        # For getting the pose
        curr_pose = self.armgroup.get_current_pose(self.end_effector_link)
        target_pose1 = Pose(curr_pose.pose.position, curr_pose.pose.orientation)
        # print(curr_pose, "\n", target_pose1)

        # Starting Postion after picking
        target_pose1.position.z += 0.06

        # lift or right
        # if random.randint(0,1):
        #     target_pose1.position.y += 0.02
        # else:
        #     target_pose1.position.y -= 0.02

        self.armgroup.set_pose_target(target_pose1)
        self.armgroup.go()

        # Open Gripper
        rospy.sleep(1.0)
        self.grippergroup.set_named_target("open")
        self.grippergroup.go()


    def goHome(self):
        # rospy.loginfo("The goHome function called")
        # For getting the pose
        curr_pose = self.armgroup.get_current_pose(self.end_effector_link)

        # Go to Home Position
        # self.attainPosition(self.pregrash_x, self.pregrash_y, self.pregrash_z)
        self.attainPosition(self.homePose.pose.position.x, self.homePose.pose.position.y, self.homePose.pose.position.z)


    def initiateGrasping(self):
        # rospy.loginfo("The initiateGrasping function called")
        rospy.sleep(3.0)
        
        
        # # 获取当前位置
        self.homePose = self.armgroup.get_current_pose(self.end_effector_link)
        # print(self.homePose)
        # self.attainPosition(0.29, -0.00296, 0.46)
    
        # 调用attainObject()函数使机械臂靠近目标
        rospy.loginfo("Approaching the Object....")
        self.attainObject()

        # 夹取物体
        rospy.loginfo("Attempting to Grasp the Object now..")
        self.grasp()

        # 夹住物体做一个小范围移动
        rospy.loginfo("Lifting the Object....")
        self.lift()

        # 机械臂返回到初始状态
        rospy.loginfo("Going back to home position....")
        self.goHome()
        rospy.sleep(2.0)

        self.grasp_running = False


if __name__ == '__main__':
    rospy.sleep(8.0) # launch文件启动时，需要在此延时，等待模型加载完成，否则节点启动失败
    # 初始化ROS节点
    rospy.init_node('simple_grasping')
    length = rospy.get_param('table_length', default=0.3)
    breadth = rospy.get_param('table_breadth', default=0.3)
    pregrasp_x = rospy.get_param('pregrasp_x', default=0.20)
    pregrasp_y = rospy.get_param('pregrasp_y', default=-0.17)
    pregrasp_z = rospy.get_param('pregrasp_z', default=0.50)
    

    simGrasp = GraspingDemo(pregrasp_x, pregrasp_y, pregrasp_z, length, breadth)
    rospy.loginfo("Waiting for two seconds..") 

    rospy.sleep(3.0)

    while not rospy.is_shutdown():
        simGrasp.initiateGrasping()
        # pass
    rospy.spin()

 
