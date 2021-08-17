#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool
from memorization.srv import String

class TestClient(object):

    def __init__(self):
        rospy.wait_for_service('memorize_target_params')
        rospy.wait_for_service('memorize_hand_poses')
        rospy.Timer(rospy.Duration(2), self.send_srv)

    def send_srv(self,event):
        print("send_srv")

        try:
            target_params = rospy.ServiceProxy('memorize_target_params', SetBool)
            resp1 = target_params(True)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        try:
            hand_poses = rospy.ServiceProxy('memorize_hand_poses', String)
            content = String()
            content.data = "rl"
            resp2 = hand_poses(content)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    print("TestClient")
    rospy.init_node("test_client")
    Test_Client_obj = TestClient()
    rospy.spin()
