#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

class TestPublisher(object):
    def __init__(self):
        self.rpub = rospy.Publisher('~slave_rarm_pose', PoseStamped, queue_size=1)
        self.lpub = rospy.Publisher('~slave_larm_pose', PoseStamped, queue_size=1)
        self.msg = PoseStamped()
        rospy.Timer(rospy.Duration(1), self.pub)

    def pub(self,event):
        self.rpub.publish(self.msg)
        self.lpub.publish(self.msg)

if __name__ == '__main__':
    print("TestPublisher")
    rospy.init_node("test_publisher")
    Test_Publisher_obj = TestPublisher()
    rospy.spin()
