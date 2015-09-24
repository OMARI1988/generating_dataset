#! /usr/bin/env python2.7

import roslib
roslib.load_manifest('baxter_demos')
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy import ndimage
from skimage import data, io, segmentation, color
from skimage.future import graph


#--------------------------------------------------------------------------------------#
class scene_segment():
    def __init__(self):
        1

    def _weight_mean_color(self,graph, src, dst, n):
        diff = graph.node[dst]['mean color'] - graph.node[n]['mean color']
        diff = np.linalg.norm(diff)
        return diff

    def merge_mean_color(self,graph, src, dst):
        graph.node[dst]['total color'] += graph.node[src]['total color']
        graph.node[dst]['pixel count'] += graph.node[src]['pixel count']
        graph.node[dst]['mean color'] = (graph.node[dst]['total color'] /
                                         graph.node[dst]['pixel count'])

    def segment_image(self,img):
        # segmentation.quickshift(img)

        labels1 = segmentation.slic(img, compactness=50, n_segments=100)
        out1 = color.label2rgb(labels1, img, kind='avg')
        cv2.imshow('test',out1)

        # g = graph.rag_mean_color(img, labels1)
        # labels2 = graph.merge_hierarchical(labels1, g, thresh=30, rag_copy=False,
        #                                in_place_merge=True,
        #                                merge_func=self.merge_mean_color,
        #                                weight_func=self._weight_mean_color)
        # # print '>>> this is easy 3'
        # self.result = color.label2rgb(labels2, img, kind='avg')
        # # print '>>> this is easy 4'
        # cv2.imshow('segmentation',self.result)
        # k = cv2.waitKey(1) & 0xFF
        # return self.result


#--------------------------------------------------------------------------------------#
class object_detection():
    def __init__(self):

        self.enlarge = 1.5
        self.depth_filter = 1000                    # filter anything more than this in mm
        self.rgb_counter = 50                     # don't accept images untill this frame for RGB
        self.d_counter = 25                     # don't accept images untill this frame for depth
        self.cv_bridge = CvBridge()	                # initilize opencv
        self.BS1 = cv2.BackgroundSubtractorMOG()    #background subtraction for xtion RGB
        self.BS2 = cv2.BackgroundSubtractorMOG()    #background subtraction for xtion depth
        self.d_mask = []

        xtion_rgb_topic = rospy.resolve_name("/camera/rgb/image_color")
        rospy.Subscriber(xtion_rgb_topic, sensor_msgs.msg.Image, self._xtion_rgb)

        xtion_depth_topic = rospy.resolve_name("/camera/depth_registered/hw_registered/image_rect_raw")
        rospy.Subscriber(xtion_depth_topic, sensor_msgs.msg.Image, self._xtion_depth)

        self.segment = scene_segment()

    def _xtion_rgb(self,imgmsg):
        self.xtion_img_rgb = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        # self.xtion_img_hsv = cv2.cvtColor(self.xtion_img_rgb, cv2.COLOR_BGR2HSV)
        # self.xtion_img_hsv[:,:,1] = 160
        # self.xtion_img_hsv[:,:,2] = 160
        if self.d_mask != []:    self.xtion_img_rgb = cv2.bitwise_and(self.xtion_img_rgb, self.xtion_img_rgb, mask = self.d_mask)
        # if self.d_mask != []:    self.xtion_img_hsv = cv2.bitwise_and(self.xtion_img_hsv, self.xtion_img_hsv, mask = self.d_mask)
        if self.rgb_counter>0:
            self.rgb_counter -= 1
        else:
            self.seg_results = self.segment.segment_image(self.xtion_img_rgb)
        cv2.imshow('xtion rgb',self.xtion_img_rgb)
        # cv2.imshow('xtion hsv',self.xtion_img_hsv)

        k = cv2.waitKey(1)

    def _xtion_depth(self,imgmsg):
        self.xtion_img_d = self.cv_bridge.imgmsg_to_cv2(imgmsg, desired_encoding="passthrough")
        self.xtion_img_d.setflags(write=True)                   # allow to change the values
        a1 = self.xtion_img_d>self.depth_filter                 # apply the depth filter1
        self.xtion_img_d[a1] = 0                                # apply the depth filter2
        self.xtion_img_d = (self.xtion_img_d.astype(float)/self.depth_filter*255).astype(int)
        fgmask = cv2.convertScaleAbs(self.xtion_img_d)          # cv2 stuff
        self.xtion_img_d_rgb = cv2.cvtColor(fgmask,cv2.COLOR_GRAY2BGR)  # cv2 stuff

        if self.d_counter>0:
            self.d_counter -= 1
        else:
            contour,hier = cv2.findContours(fgmask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contour:
                cv2.drawContours(fgmask,[cnt],0,255,-1)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(10,10))
            fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN,kernel)
            fgmask = cv2.bitwise_not(fgmask)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(30,30))
            fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN,kernel)
            self.d_mask = cv2.bitwise_not(fgmask)


    def _subtract(self, img, BS):
        fgmask = BS.apply(img)
        contour,hier = cv2.findContours(fgmask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contour:
            cv2.drawContours(fgmask,[cnt],0,255,-1)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN,kernel)
        fgmask = cv2.bitwise_not(fgmask)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(30,30))
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN,kernel)
        fgmask = cv2.bitwise_not(fgmask)
        return fgmask

    def _plot_results(self, img, fgmask):
        contour,hier = cv2.findContours(fgmask,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        for count,cnt in enumerate(contour):
            area = cv2.contourArea(cnt)
            x,y,w,h = cv2.boundingRect(cnt)
            cx = x+w/2
            cy = y+h/2
            cw = int(w/2*self.enlarge)
            ch = int(h/2*self.enlarge)
            cv2.rectangle(img, (cx-cw,cy-ch), (cx+cw,cy+ch), (255*np.random.rand(3)).astype(int), 2)
            cv2.line(img,(cx-10,cy), (cx+10,cy), (255*np.random.rand(3)).astype(int), 2)
            cv2.line(img,(cx,cy-10), (cx,cy+10), (255*np.random.rand(3)).astype(int), 2)
            #roi=img[y:y+h,x:x+w]
            #cv2.imshow(str(count),roi)
        return img




#--------------------------------------------------------------------------------------#
def main():
    object_detection()

    rospy.init_node('object_detection')
    rospy.loginfo('Object detection running..')
    rospy.spin()

#--------------------------------------------------------------------------------------#
if __name__ == '__main__':
    main()
