#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
import cv_bridge
import cv2
from jsk_recognition_utils.depth import split_fore_background
# from jsk_topic_tools import ConnectionBasedTransport
import rospy
from sensor_msgs.msg import Image
import dynamic_reconfigure.server
# from near_far_params.cfg import near_far_params
# from memorization.cfg import nearfarparams
from memorization.cfg import NearFarParamsConfig

class get_mask_image():
    def __init__(self):
        self.srv = dynamic_reconfigure.server.Server(NearFarParamsConfig, self.dynamic_reconfigure_callback)
        self.near = rospy.get_param("~near", 0.)
        self.far = rospy.get_param("~far", 3.)
        self.mask_img=None
        self.bridge = cv_bridge.CvBridge()
        self.sub_depth_image = rospy.Subscriber('~input/depth_image', Image, self.cb_depth_image)
        self.sub_color_image = rospy.Subscriber('~input/color_image', Image, self.cb_color_image)
        
        # self.mask_pub_ = self.advertise('~output/mask_image', Image, queue_size=10)
        # self.masked_pub_ = self.advertise('~output/masked_image', Image, queue_size=10)
        self.mask_pub_ = rospy.Publisher('~output/mask_image', Image, queue_size=10)
        self.masked_pub_ = rospy.Publisher('~output/masked_image', Image, queue_size=10)

    def dynamic_reconfigure_callback(self, config, level):
        self.near = config["near"]
        self.far = config["far"]
        return config
    
    def cb_depth_image(self,msg):
                
        depth_img = self.bridge.imgmsg_to_cv2(msg)
        # depth_gray = cv2.cvtColor(depth, cv2.COLOR_BGR2GRAY)
        ret,self.mask_img = cv2.threshold(depth_img,self.near,self.far,cv2.THRESH_BINARY)

        mask_msg = bridge.cv2_to_imgmsg(self.mask_img, encoding='mono8')
        mask_msg.header = msg.header
        self.mask_pub_.publish(mask_msg)
        # if msg.encoding == '32FC1':
        #     # convert float to int (handle 32FC1 as 16UC1)
        #     nan_mask = np.isnan(depth)
        #     depth[] = 0
        #     depth *= 255
        #     depth = depth.astype(np.uint8)
    def cb_color_image(self,msg):
        color_img = self.bridge.imgmsg_to_cv2(msg)
        color_img[self.mask_img==0]=0
        mask_msg = bridge.cv2_to_imgmsg(color_img, encoding='mono8')
        mask_msg.header = msg.header
        self.masked_pub_.publish(mask_msg)


if __name__ == '__main__':
    rospy.init_node('get_masked_image')
    mask_img_instance = get_mask_image()
    rospy.spin()        
        
