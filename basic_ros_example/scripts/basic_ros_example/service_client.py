#! /usr/bin/env python3
import rospy
from basic_ros_example.srv import Adder

rospy.init_node('service_client')

adder_srv = rospy.ServiceProxy('adder_service', Adder)
adder_srv.wait_for_service()

try:
    response = adder_srv(val1=4.5, val2=1.0)
    rospy.loginfo(f'Service succeeded! Result: {response.result}')
except rospy.ServiceException as e:
    rospy.logwarn(f'Service call failed! {e}')