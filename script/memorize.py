#!/usr/bin/env python
import rospy
import numpy as np
from jsk_recognition_msgs.msg import BoundingBoxArray
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import SetBool
from memorization.srv import String

class Memorize(object):
    def __init__(self):
        rospy.Subscriber('~bbox', BoundingBoxArray, self.get_params)
        rospy.Subscriber('~slave_rarm_pose', PoseStamped, self.get_rhand_pose)
        rospy.Subscriber('~slave_larm_pose', PoseStamped, self.get_lhand_pose)

        self.get_target_params_srv = rospy.Service('memorize_target_params', SetBool, self.memorize_target_params)
        self.get_hand_poses_srv = rospy.Service('memorize_hand_poses', String, self.memorize_hand_poses)
        self.center = []
        self.r = []
        self.tgt_num = 1
        self.rhand_pose = PoseStamped()
        self.lhand_pose = PoseStamped()
        self.memory = {"center": self.center, "radious": self.r, "rhand_pose": self.rhand_pose, "lhand_pose": self.lhand_pose}
        
    def get_params(self,msg):
        boxes = msg.boxes
        boxes_list =np.array([[0,0,0]])
        dimension_list = np.array([[0,0,0]])
        for box in boxes:
            boxes_list = np.append(boxes_list,np.array([[box.pose.position.x, box.pose.position.y, box.pose.position.z]]),axis=0)
            dimension_list = np.append(dimension_list,np.array([[box.dimensions.x, box.dimensions.y, box.dimensions.z]]),axis=0)
        boxes_list = boxes_list[1:]
        dimension_list = dimension_list[1:]
        sorted_index = np.argsort(map(lambda x :abs(x[0]*x[1]*x[2]),dimension_list)[::-1])[:self.tgt_num] #extract self.tgt_num
        boxes_list = boxes_list[sorted_index]
        dimension_list = dimension_list[sorted_index]
        self.center = boxes_list
        self.r = map(lambda x : abs(x[0]) / 2.0 ,dimension_list)
        print("center: {}, r: {}".format(self.center,self.r))

    def get_rhand_pose(self,msg):
        # print("get_rhand_pose")
        self.rhand_pose = msg.pose

    def get_lhand_pose(self,msg):
        # print("get_lhand_pose")
        self.lhand_pose = msg.pose

    def memorize_hand_poses(self,req):
        print("memorize_hand_poses")
        if "r" in str(req.data):
            self.memory["rhand_pose"] = self.rhand_pose
            print("memorize rhand_pose")
        if "l" in str(req.data):
            self.memory["lhand_pose"] = self.lhand_pose
            print("memorize lhand_pose")

    def memorize_target_params(self,req):
        print("memorize params: center: {}, r: {}".format(self.memory["center"], self.memory["radious"]))
        self.memory["center"] = self.center
        self.memory["radious"] = self.r

if __name__ == '__main__':
    print("OK")
    rospy.init_node("Memorize")
    Memorize_obj = Memorize()
    rospy.spin()
