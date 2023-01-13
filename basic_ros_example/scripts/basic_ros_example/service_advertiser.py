#! /usr/bin/env python3
import rospy
from basic_ros_example.srv import Adder, AdderResponse


class ServiceAdvertiser:
    def __init__(self):
        self.srv = rospy.Service('adder_service', Adder, self.srv_callback)

    def srv_callback(self, req):
        res = AdderResponse()
        res.result = req.val1 + req.val2
        return res


if __name__ == '__main__':
    rospy.init_node('service_advertiser')
    node_instance = ServiceAdvertiser()
    rospy.spin()