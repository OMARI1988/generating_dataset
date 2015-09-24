#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('dataset_creation')
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np

#--------------------------------------------------------------------------------------#
class object_detection():

    def __init__(self):
        self.saved = 0
        self.cv_bridge = CvBridge()	                # initilize opencv
        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color")
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

    def _xtion_rgb(self,imgmsg):
        self.xtion_img = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

        self.xtion_hsv = cv2.cvtColor(self.xtion_img, cv2.COLOR_BGR2HSV)

        cv2.imshow('xtion rgb',self.xtion_img)
        cv2.imshow('xtion hsv',self.xtion_hsv)
        k = cv2.waitKey(1) & 0xff

#--------------------------------------------------------------------------------------#
def main():
    object_detection()
    rospy.init_node('cluster_image_colours')
    rospy.loginfo('cluster image colours..')
    while not rospy.is_shutdown():
        pass
#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()
