#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
from probot_demo.srv import Img_process, Img_processRequest
from probot_demo.msg import Process_Result


if __name__ == "__main__":
	
    # ROS节点初始化
    rospy.init_node('image_client')

	# 发现/image_process/next服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/image_process/next')
    try:
        image_client = rospy.ServiceProxy('/image_process/next', Img_process)
        # 请求服务调用，输入请求数据
        request = Img_processRequest()
        request.signal = ['init', 'blue', 'green', 'red']
        response = image_client(request)
        
        #服务调用并显示调用结果
        print "Show result :"
        print response.result
        # print response.objects
        for obj in response.objects:
            print obj 
            print 'next'
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
       


