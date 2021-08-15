#!/usr/bin/env python
import rospy
import numpy as np
import message_filters
import tf2_ros
import tf2_geometry_msgs
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import Polygon #test
from geometry_msgs.msg import PoseStamped

from jsk_recognition_msgs.msg import BoundingBoxArray

class GetGrabPose(object):
    def __init__(self):
        rospy.Subscriber('~bbox', BoundingBoxArray, self.get_params)
        self.center = []
        self.r = []
        self.tgt_num = 1
        self.memory = {}
        
    def get_params(self,msg):
        boxes = msg.boxes
        boxes_list =np.array([[]])
        dimension_list = np.array([[]])
        for box in boxes:
            boxes_list = np.append(boxes_list,np.array([[box.pose.position.x, box.pose.position.y, box.pose.position.z]],axis=0))
            dimension_list = np.append(dimension_list,np.array([[box.pose.dimension.x, box.pose.dimension.y, box.pose.dimension.z]],axis=0))
        sorted_index = np.argsort(map(lambda x :abs(x[0]*x[1]*x[2]),tmp)[::-1])[:self.tgt_num] #extract self.tgt_num
        boxes_list = boxes_list[sorted_index]
        dimension_list = dimension_list[sorted_index]
        self.center = boxes_list
        self.r = map(lambda x : abs(x[0]) / 2.0 ,dimension_list)

    def memorize(self):
        print("memorize params\n")
        self.memory = {"center": self.center, "radious": self.r}
        
                
if __name__ == '__main__':
    print("OK")
    rospy.init_node("PutPointsOnImage")
    PutPointsOnImage_obj = PutPointsOnImage()
    rospy.spin()
