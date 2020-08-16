#!/usr/bin/env python2
# -*- coding: utf-8 -*-

from __future__ import print_function
import sys
import rospy

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')

class image_listenner:

    def __init__(self): 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        self.image_id = 150

    def callback(self, data):
        ''' callback of image_sub '''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv2.imshow("Image window", cv_image)
            c = cv2.waitKey(10)
            # print(c)
            if c == 115:  # s
                print('save image {}'.format(self.image_id))
                cv2.imwrite('../image/image_{}.jpg'.format(self.image_id), cv_image)
                self.image_id += 1 
        except CvBridgeError as e:
            print(e) 

 
if __name__ == '__main__':
    image_listenning = image_listenner()
    rospy.init_node('image_listenner', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
    
    
