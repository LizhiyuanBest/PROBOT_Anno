#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import Bool, Float32

if __name__ == '__main__':
    rospy.init_node('actuator', anonymous=True)
    # 吸盘控制
    suction_pub = rospy.Publisher('suction_chatter',Bool,queue_size=10)
    # 夹爪控制
    gripper_pub = rospy.Publisher('gripper_chatter',Bool,queue_size=10)
    # 定义Bool变量 并初始化
    suction_signal = Bool(data=False)
    gripper_signal = Bool(data=False)
    
    # 下面使吸盘和夹爪间歇切换工作
    while True:
        # 发布False,夹爪张开
        gripper_signal.data = False
        gripper_pub.publish(gripper_signal)
        print("stop")
        rospy.sleep(2)
        # 发布True，夹爪闭合
        gripper_signal.data = True
        gripper_pub.publish(gripper_signal)
        print("activate")
        rospy.sleep(2)

        # 发布False,吸盘释放
        suction_signal.data = False
        suction_pub.publish(suction_signal)
        print("stop")
        rospy.sleep(2)
        # 发布True，吸盘吸取
        suction_signal.data = True
        suction_pub.publish(suction_signal)
        print("activate")
        rospy.sleep(2)
