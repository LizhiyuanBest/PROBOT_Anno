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
from std_msgs.msg import Float32MultiArray, String
import time
usleep = lambda x: time.sleep(x/1000000.0)  # python 当中没有usleep

dx = dy = float(400) / 290  # mm/像素  1.37931
pi = 3.141592

u0 = 400
v0 = 400

# 不同物体抓手的闭合程度不同 值越大，爪子越紧
cube_grip = 0.59
cuboid_grip = 0.60
cylinder_grip = 0.53
high_init = 0.06
y_init = 0.25
cube_x = 0.0
cuboid_x = 0.09
cylinder_x = 0.185
# 每个物体的放置位置
grasp_order = [['green_cube', [cube_x, y_init, high_init+0.03*0], cube_grip+0.005], #
            ['green_cuboid', [cuboid_x, y_init, high_init+0.02*0], cuboid_grip+0.005],
            ['green_cylinder', [cylinder_x, y_init, high_init+0.04*0], cylinder_grip-0.005],
            ['blue_cube', [cube_x, y_init, high_init+0.03*1], cube_grip+0.018], #
            ['blue_cuboid', [cuboid_x, y_init, high_init+0.02*1], cuboid_grip+0.005],
            ['blue_cylinder', [cylinder_x, y_init, high_init+0.04*1], cylinder_grip],
            ['red_cube', [cube_x, y_init, high_init+0.03*2], cube_grip+0.008], #
            ['red_cuboid', [cuboid_x, y_init, high_init+0.02*2], cuboid_grip+0.003], #
            ['red_cylinder', [cylinder_x, y_init, high_init+0.04*2], cylinder_grip-0.005]]

def decode_res(objects):
    """ 解码数据, 按颜色分类 """
    objs = {}
    objs['red'] = []
    objs['blue'] = []
    objs['green'] = []
    for obj in objects:
        region = [] # 区域信息
        for pt in obj.region:
            region.append([pt.x, pt.y])
        objs[obj.color].append([obj.type,region, obj.center, obj.theta])
    return objs

def obj_info(objects):
    """ 解码数据, 按目标分类, 重复目标不计入 """
    # print(objects)
    objs = {}
    for obj in objects:
        if obj.type == 'cube': # 选择正方体小的角度
            print('cube',obj.theta)
            if obj.theta > 45:
                obj.theta -= 90
            elif obj.theta < -45:
                obj.theta += 90
        if obj.type == 'cuboid': # 将长方体的角度固定
            dy1 = obj.region[1].y - obj.region[0].y
            dx1 = obj.region[1].x - obj.region[0].x
            dy2 = obj.region[2].y - obj.region[1].y
            dx2 = obj.region[2].x - obj.region[1].x
            # k1 = math.atan2(dy1, dx1) / pi * 180
            # k2 = math.atan2(dy2, dx2) / pi * 180 
            # print('k1',k1,'k2',k2)
            if (dx1*dx1 + dy1*dy1) > (dx2*dx2 + dy2*dy2):
                k = math.atan2(dy1, dx1) / pi * 180 + 90
                # print('k1',k)
            else:
                k = math.atan2(dy2, dx2) / pi * 180 + 90
                # print('k2',k)
            # print(obj.color+'_'+obj.type, obj.theta, k)
            obj.theta = k
        if obj.type == 'cylinder': # 将圆柱体的高度考虑进来
            # print(1-0.04/0.7)  # (1-h/H)=0.942857142857
            # x=(x-400)(1-0.04/0.7) + 400
            obj.center = list(obj.center)
            obj.center[0] = (obj.center[0] - 400) * 0.942857142857 + 400
            obj.center[1] = (obj.center[1] - 400) * 0.942857142857 + 400
            # print(obj.color+'_'+obj.type, obj.center,[x,y])
        if obj.color+'_'+obj.type not in objs:  # 第一次存入
            objs[obj.color+'_'+obj.type] = [obj.center, obj.theta]
        # elif obj.color+'_'+obj.type in objs:  # 第二次 设置为空会导致读取空列表而出错，暂时先不处理
        #     objs[obj.color+'_'+obj.type] = [ ]
    return objs

def get_2D_location(pt):
    """得到相机坐标系下的2D平面坐标, ros 中单位为米"""
    # print pt[0], pt[1]
    # print dx, dy
    x = (pt[0] - u0) * dx
    y = (pt[1] - v0) * dy
    return x/1000, y/1000

def init_again(center):
    """ 更改初始配置 """
    cent = get_2D_location(center)
    print(cent)
    global y_init
    global cube_x
    global cuboid_x
    global grasp_order
    # 图像xy与实际平面xy相反
    y_init = 0.25 + cent[0]
    cube_x = 0.03 + cent[1] 
    cuboid_x = 0.12 + cent[1] 
    cylinder_x = 0.215 + cent[1] 

    grasp_order = [['green_cube', [cube_x, y_init, high_init+0.03*0], cube_grip+0.005], #
            ['green_cuboid', [cuboid_x, y_init, high_init+0.02*0], cuboid_grip+0.005],
            ['green_cylinder', [cylinder_x, y_init, high_init+0.04*0], cylinder_grip-0.005],
            ['blue_cube', [cube_x, y_init, high_init+0.03*1], cube_grip+0.018], #
            ['blue_cuboid', [cuboid_x, y_init, high_init+0.02*1], cuboid_grip+0.005],
            ['blue_cylinder', [cylinder_x, y_init, high_init+0.04*1], cylinder_grip],
            ['red_cube', [cube_x, y_init, high_init+0.03*2], cube_grip+0.008], #
            ['red_cuboid', [cuboid_x, y_init, high_init+0.02*2], cuboid_grip+0.003], #
            ['red_cylinder', [cylinder_x, y_init, high_init+0.04*2], cylinder_grip-0.005]]

class Robot:

    def __init__(self, pregrasp):
        # ROS节点初始化
        rospy.init_node('image_client', anonymous=True)
        self.pregrash = pregrasp  # 准备抓取的姿势
        self.grasp_color = ['green', 'red', 'blue']  # ['green', 'red', 'blue']
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.arm = moveit_commander.MoveGroupCommander('manipulator')
        # 当运动规划失败后，允许重新规划
        self.arm.allow_replanning(True)
        # 设置目标位置所使用的参考坐标系
        self.reference_frame = 'base_link'
        self.arm.set_pose_reference_frame(self.reference_frame)
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.arm.set_goal_position_tolerance(0.001)
        self.arm.set_goal_orientation_tolerance(0.001)
        # 设置允许的最大速度和加速度
        self.arm.set_max_acceleration_scaling_factor(1)
        self.arm.set_max_velocity_scaling_factor(1)
        # 获取终端link的名称
        self.end_effector_link = self.arm.get_end_effector_link()
        # 控制机械臂先回到初始化位置
        self.arm.set_named_target('home')
        self.arm.go()
        self.grippergroup = moveit_commander.MoveGroupCommander('gripper')
        self.grippergroup.set_named_target("open")
        self.grippergroup.go()
        
        ############## 创建一些控制真实机器人的话题通信
        # 创建一个发布者 speed_pub ，发布消息类型为std_msgs::Float32MultiArray，话题名为 speed_chatter ，用于发布速度
        # ros::Publisher speed_pub = node_handle.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000)
        self.speed_pub = rospy.Publisher('speed_chatter', Float32MultiArray, queue_size=1000)
        # 创建一个发布者 position_pub ，发布消息类型为std_msgs::Float32MultiArray，话题名为 position_chatter ，用于发布位置
        # ros::Publisher position_pub = node_handle.advertise<std_msgs::Float32MultiArray>("position_chatter", 1000)
        self.position_pub = rospy.Publisher('position_chatter', Float32MultiArray, queue_size=1000)
        # 创建一个订阅者Feedback_sub，接收话题"Feedback_chatter"，回调函数为Feed_back
        # ros::Subscriber Feedback_sub = node_handle.subscribe("Feedback_chatter",100,Feed_back)
        self.feedback_sub = rospy.Subscriber('Feedback_chatter', String, callback=self.feed_back, queue_size=100)
        self.Feedback = 1
        # self.real_go_home()  # 有问题
        ############## 创建一些控制真实机器人的话题通信
        
        rospy.loginfo("Getting into the Grasping Position....")
        # 调用该函数控制机械臂运动到设定的位置
        self.attainPosition(pregrasp)
        '''
            # # 得到相机和机器人的tf关系 简单的情况下可以自己去算，在此就不用tf了
            # self.tf_camera_to_robot = tf.TransformListener()
            # got_tf = False
            # while not got_tf:
            #     try: # 如果查询得到的话，就将结果保存到camera_to_robot_，保存x,y,z和四元数一共7个值
            #         (self.trans,self.rot) = self.tf_camera_to_robot.lookupTransform("/base_link", "/camera_link", rospy.Time(0))
            #         rospy.loginfo("[adventure_tf]: got")
            #         # print("trans ",self.trans)
            #         # print("rot ",self.rot)
            #         got_tf = True
            #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #         rospy.loginfo("[adventure_tf]: (wait)")
            #         rospy.sleep(1.0)
        '''
        # 发现/image_process/next服务后，创建一个服务客户端，连接名为/image_process/next的service
        rospy.wait_for_service('/image_process/next')
        self.image_client = rospy.ServiceProxy('/image_process/next', Img_process)
        rospy.sleep(0.2)
        self.next_objs(['init'])

    def feed_back(self, Feedback_msg):
        """ 真实机械臂返回消息表示指令执行完成 """
        self.Feedback = 0

    def real_go_home(self):
        ######################  控制真实机器人需要
        # 这个还有问题，不可以用
        position_msg = Float32MultiArray()
        for _ in range(7):
            position_msg.data.append(0.0)
        # 设置角度值
        position_msg.data[0] = 0  # joint_positions[0] * 30 * 180 / pi
        position_msg.data[1] = 0  # joint_positions[1] * 205 * 180 / (3 * pi)
        position_msg.data[2] = 0  # joint_positions[2] * 50 * 180 / pi
        position_msg.data[3] = 0  # joint_positions[3] * 125 * 180 / (2 * pi)
        position_msg.data[4] = 0  # joint_positions[4] * 125 * 180 / (2 * pi)
        position_msg.data[5] = 0  # joint_positions[5] * 200 * 180 / (9 * pi)
        position_msg.data[6] = 800 # 整体速度值
        # 发送角度值
        self.position_pub.publish(position_msg)
        ######################  控制真实机器人需要

    def attainPosition(self, pt, angle=None):
        # rospy.loginfo("The attain position function called")
        if angle :
            curr_joints = self.arm.get_current_joint_values()
            # print("curr_joints", curr_joints)
            joint_positions = curr_joints
            joint_positions[5] += pi*float(angle)/180
            self.arm.set_joint_value_target(joint_positions)
            self.arm.go() # 机械臂运动

            ######################  控制真实机器人需要
            position_msg = Float32MultiArray()
            for _ in range(7):
                position_msg.data.append(0.0)
            # 设置角度值
            position_msg.data[0] = joint_positions[0] * 30 * 180 / pi
            position_msg.data[1] = joint_positions[1] * 205 * 180 / (3 * pi)
            position_msg.data[2] = joint_positions[2] * 50 * 180 / pi
            position_msg.data[3] = joint_positions[3] * 125 * 180 / (2 * pi)
            position_msg.data[4] = joint_positions[4] * 125 * 180 / (2 * pi)
            position_msg.data[5] = joint_positions[5] * 200 * 180 / (9 * pi)
            position_msg.data[6] = 800 # 整体速度值
            # 发送角度值
            self.position_pub.publish(position_msg)
            ######################  控制真实机器人需要

        # 获取当前位置
        curr_pose = self.arm.get_current_pose()
        #print("curr_pose", curr_pose)
        target_pose = copy.copy(curr_pose)
        # 设置抓取前的机械臂位置
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        # target_pose = PoseStamped()
        # target_pose.header.frame_id = self.reference_frame  # There are some errors, and I don't know why
        target_pose.header.stamp = rospy.Time.now() 
        target_pose.pose.position.x = pt[0]
        target_pose.pose.position.y = pt[1]
        target_pose.pose.position.z = pt[2]
        # target_pose.pose.orientation.x = 0.000167651094588
        # target_pose.pose.orientation.y = 0.706901698875
        # target_pose.pose.orientation.z = -6.66379198814e-05
        # target_pose.pose.orientation.w = 0.707311781027
        # print("target_pose", target_pose)
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        self.arm.set_pose_target(target_pose, self.end_effector_link)
        
        # 规划运动路径
        traj = self.arm.plan()
        # print(traj)
        
        # 按照规划的运动路径控制机械臂运动
        self.arm.execute(traj)
        # #################  # 控制真实机械臂需要
        # self.speed_msg_pub(traj)
        # #################  # 控制真实机械臂需要
        # rospy.sleep(1)

        # self.arm.set_pose_target(target_pose)
        # self.arm.go() # 机械臂运动

    def grip_r(self, angle):
        """ 第六轴旋转 """
        curr_joints = self.arm.get_current_joint_values()
        # print("curr_joints", curr_joints)
        joint_positions = curr_joints
        joint_positions[5] = pi*float(angle)/180
        self.arm.set_joint_value_target(joint_positions)
        self.arm.go() # 机械臂运动

    def next_objs(self, signal):
        """ 调用服务，获取物体信息 """
        objs = decode_res([])
        try:
            # 请求服务调用，输入请求数据
            request = Img_processRequest()
            request.signal = signal # ['blue', 'green', 'red']
            # 服务调用并显示调用结果
            response = self.image_client(request)
            print "Show result : %s" % response.result
            if response.result == 'ok':
                # print response.objects
                objs = obj_info(response.objects)
                # print objs
            elif response.result == 'init':
                print response.objects[0].center
                init_again(response.objects[0].center) # 修改一些配置参数
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return objs

    def get_3D_location(self, pt):
        """ 得到机器人坐标系下的3D坐标 """
        xc,yc = get_2D_location(pt)
        # print ('2D location: xc={:.4f}, yc={:.4f}'.format(xc,yc))
        '''
            # # 通过坐标变换，将二维坐标变换为相机坐标系下的三维坐标，在本程序中与URDF建模有关系
            # obj_camera_frame  = PointStamped(header=rospy.Header(
            #     stamp = rospy.get_rostime(),frame_id="/camera_link"), point=Point(x=0.6,y=-xc,z=-yc))
            # # 关键的一行代码，将相机坐标系下的位置转化为base_link坐标系下的坐标
            # obj_robot_frame = self.tf_camera_to_robot.transformPoint(target_frame="/base_link", ps=obj_camera_frame)
            # print(" X-Co-ordinate in Robot Frame : ", obj_robot_frame.point.x)
            # print(" Y-Co-ordinate in Robot Frame : ", obj_robot_frame.point.y)
            # print(" Z-Co-ordinate in Robot Frame : ", obj_robot_frame.point.z)
            # 上面注释的等价于下面这几行，上面是通过ROS的TF关系求得的，下面是自己算的，TF关系包含在里面了
        '''
        x = 0.3 - yc  # 具体的参数和建模有关系，可以查看robot和camera的urdf文件
        y = -xc
        z = 0.7 - 0.6
        # print ('3D location: x={:.4f}, y={:.4f}, z={:.4f}'.format(x,y,z))
        return [x,y,z]

    def get_3D_locations(self, objs):
        """ 得到所有物体的3D坐标和角度 """
        res = {}
        for type_obj, obj in objs.items():
            loc = self.get_3D_location(obj[0])
            res[type_obj] = [loc, obj[1]]
        return res

    def gripper_close(self, value=0.6):
        """ 夹爪关闭 """
        # self.grippergroup.set_named_target("close")
        self.grippergroup.set_joint_value_target([value])  # box: 0.55  # cylinder 0.52
        self.grippergroup.go()

    def gripper_open(self):
        """ 夹爪打开 """
        # self.grippergroup.set_named_target("open")
        self.grippergroup.set_joint_value_target([0.3])  
        self.grippergroup.go()

    def pick(self, obj, grip):
        """ 到达目标位置 """
        # self.attainPosition(obj[0],obj[1])
        # Open Gripper
        # rospy.sleep(0.1)
        # self.grippergroup.set_named_target("open")
        # self.grippergroup.go()
        print('obj', obj)
        obj[0][2] -= 0.054
        # obj[0][0] += 0.01
        self.attainPosition(obj[0], obj[1])
        self.gripper_close(grip) # 爪子闭合
        obj[0][2] += 0.06  # 抓手抬起来
        self.attainPosition(obj[0])
        self.attainPosition(obj[0], -obj[1])
        # self.grip_r(0)

    def place(self, obj):
        """ 到达目标位置 """
        print('obj', obj)
        pos = copy.deepcopy(obj)
        pos[2] -= 0.01
        # pos[0] += 0.01
        self.attainPosition(pos)
        self.gripper_open() # 爪子打开，释放物体
        pos[2] += 0.05  # 抓手抬起来
        self.attainPosition(pos)

    def track(self, obj):
        """ 中间点暂时定在准备点，经过中间点插值 """
        # print obj

        # 获取当前位姿数据最为机械臂运动的起始位姿
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # print start_pose
                
        # 初始化路点列表
        waypoints = []
                
        # 将初始位姿加入路点列表
        waypoints.append(start_pose)
        wait_point = [0.2,0.25,0.2]
        # 设置路点数据，并加入路点列表
        wpose = copy.deepcopy(start_pose)

        wpose.position.z = wait_point[2]
        waypoints.append(copy.deepcopy(wpose))
        
        wpose.position.x = wait_point[0] # self.pregrash[0]
        wpose.position.y = wait_point[1] # self.pregrash[1]
        wpose.position.z = wait_point[2] # self.pregrash[2]
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = obj[0]
        wpose.position.y = obj[1]
        wpose.position.z = wait_point[2]
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = obj[0]
        wpose.position.y = obj[1]
        wpose.position.z = obj[2]
        waypoints.append(copy.deepcopy(wpose))
        
        # wpose.position.x = obj[0]
        # wpose.position.y = obj[1]
        # wpose.position.z = obj[2]
        # waypoints.append(copy.deepcopy(wpose))

        fraction = 0.0   #路径规划覆盖率
        maxtries = 100   #最大尝试规划次数
        attempts = 0     #已经尝试规划次数
        
        # 设置机器臂当前的状态作为运动初始状态
        self.arm.set_start_state_to_current_state()
 
        # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = self.arm.compute_cartesian_path(
                                    waypoints,   # waypoint poses，路点列表
                                    0.01,        # eef_step，终端步进值
                                    0.0,         # jump_threshold，跳跃阈值
                                    True)        # avoid_collisions，避障规划
            
            # 尝试次数累加
            attempts += 1
            
            # 打印运动规划进程
            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        # print(plan)

        # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            self.arm.execute(plan)
            ##################  # 控制真实机械臂需要
            # self.speed_msg_pub(plan)
            ##################  # 控制真实机械臂需要
            rospy.loginfo("Path execution complete.")
        # 如果路径规划失败，则打印失败信息
        else:
            rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        # rospy.sleep(1)

    def speed_msg_pub(self, plan):
        ##################  # 控制真实机械臂需要
        rospy.loginfo("Path computed successfully. Moving the really arm.")
        # print(plan.joint_trajectory)
        start = time.time() # 开始时间
        speed_msg = Float32MultiArray()
        for i in range(6):
            speed_msg.data.append(0.0)
        for i in range(len(plan.joint_trajectory.points)):
            # 设置速度值
            speed_msg.data[0] = plan.joint_trajectory.points[i].velocities[0] * 30 * 180 / pi
            speed_msg.data[1] = plan.joint_trajectory.points[i].velocities[1] * 205 * 180 / (3 * pi)
            speed_msg.data[2] = plan.joint_trajectory.points[i].velocities[2] * 50 * 180 / pi
            speed_msg.data[3] = plan.joint_trajectory.points[i].velocities[3] * 125 * 180 / (2 * pi)
            speed_msg.data[4] = plan.joint_trajectory.points[i].velocities[4] * 125 * 180 / (2 * pi)
            speed_msg.data[5] = plan.joint_trajectory.points[i].velocities[5] * 200 * 180 / (9 * pi)
            # 发送速度值
            self.speed_pub.publish(speed_msg)

            # 计算这个速度持续时间
            time_start = plan.joint_trajectory.points[i].time_from_start.to_sec()
            if i < len(plan.joint_trajectory.points)-1 :
                time_end = plan.joint_trajectory.points[i+1].time_from_start.to_sec()
                time_duration = (time_end - time_start) * 1000000 - 250   # 假定每个循环的时间花费250微秒
            else:
                time_duration = 10000

            usleep(time_duration)

        end = time.time()
        print("Running time: {:.5f}s".format(end - start))
        ##################  # 控制真实机械臂需要

    def once_obj(self, objs):
        """ once object """
        # print objs
        objs_loc = self.get_3D_locations(objs)
        if len(objs_loc) > 0:  # 检测到待抓物体
            for gr in grasp_order:
                if gr[0] in objs_loc:
                    # print gr[0]
                    self.track(objs_loc[gr[0]][0])
                    self.pick(objs_loc[gr[0]], gr[2])
                    self.track(gr[1])
                    self.place(gr[1])
                    break
        else:
            print("loop end")     

    def once_loop(self):
        """ once loop """
        objs = self.next_objs(self.grasp_color)
        # print objs
        while len(objs) > 0:  # 检测到待抓物体
            self.once_obj(objs)
            objs = self.next_objs(self.grasp_color)

        print("loop end") 


if __name__ == "__main__":
	
    # rospy.sleep(2.0)
    rb = Robot([0.1, 0.25, 0.2])  # 创建机器人实例
    # rospy.sleep(1.0)
    rb.gripper_open()
    rb.once_loop()
    # rb.attainPosition([0.47,-0.001,0.1])


# {'red_cuboid': [(399.0, 399.0), -3.814074993133545], 
# 'green_cuboid': [(399.25, 473.5), -42.87890625], 
# 'blue_cuboid': [(473.5, 324.5), -32.47119140625]}