#!/home/li/anaconda3/bin python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy
import numpy as np
import os
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nn_vs.srv import Save_image, Save_imageResponse
 
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')


ROOT_PATH = '/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images'


class image_listenner:

    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/probot_anno/hand_camera/hand_image_raw",Image,self.image_sub_callback)
        self.image_srv = rospy.Service('/image_save', Save_image, self.image_srv_callback)
        self.image_id = 210
        self.img = np.zeros((640, 480, 3), dtype=np.uint8)  # 初始图像

    def image_sub_callback(self, data):
        ''' callback of image_sub '''
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", self.img)
            c = cv2.waitKey(10)
            # print(c)
            if c == 115:  # s
                print('save image {}'.format(self.image_id))
                cv2.imwrite('/home/li/ROS/probot_ws/src/PROBOT_Anno/nn_vs/images/image_{}.jpg'.format(self.image_id), self.img)
                self.image_id += 1 
        except CvBridgeError as e:
            print(e) 

    def image_srv_callback(self, data):
        print('save image {}'.format(data.file_name))
        cv2.imwrite(os.path.join(ROOT_PATH, data.file_name), self.img)
        self.image_id += 1 
        res = Save_imageResponse()
        res.result = 'ok'
        return res

if __name__ == '__main__':
    image_listenning = image_listenner()
    rospy.init_node('image_listenner', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
    