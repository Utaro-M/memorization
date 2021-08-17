#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool

class TestClient(object):

    def __init__(self):
        rospy.wait_for_service('memorize_target_params')
        rospy.Timer(rospy.Duration(2), self.send_srv)

    def send_srv(self,event):
        print("send_srv")
        try:
            target_params = rospy.ServiceProxy('memorize_target_params', SetBool)
            resp1 = target_params(True)
            return resp1.success
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':
    print("TestClient")
    rospy.init_node("test_client")
    Test_Client_obj = TestClient()
    rospy.spin()
