#! /usr/bin/env python3
import rospy

rospy.init_node('static_param_example')

p1 = rospy.get_param('p1', default=5.0)
rospy.loginfo(f'p1: {p1}')
p2 = rospy.get_param('~p2', default=8.0)
rospy.loginfo(f'p2: {p2}')

try:
    p3 = rospy.get_param('p3')
    rospy.loginfo(f'p3: {p3}')
except:
    rospy.logwarn('Could not find parameter p3')