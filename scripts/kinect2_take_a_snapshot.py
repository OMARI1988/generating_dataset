#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('generating_dataset')
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
        self.BS1 = cv2.BackgroundSubtractorMOG()    #background subtraction for xtion RGB
        xtion_rgb_topic = rospy.resolve_name("/kinect2/hd/image_color")
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

    def _xtion_rgb(self,imgmsg):
        self.xtion_img = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")

        cv2.imshow('xtion rgb',self.xtion_img)
        cv2.imwrite('/home/omari/Datasets/Language_and_vision_Demo/testing/snapshot.png',self.xtion_img)
        k = cv2.waitKey(1) & 0xff
        self.saved = 1

#--------------------------------------------------------------------------------------#
def main():
    object_detection()

    rospy.init_node('take_a_snapshot')
    rospy.loginfo('taking a single snapshot..')
    while not rospy.is_shutdown():
        pass
        # if object_detection.saved:
            # break

#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()
