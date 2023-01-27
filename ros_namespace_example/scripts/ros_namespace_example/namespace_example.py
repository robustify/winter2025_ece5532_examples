#! /usr/bin/env python3
import rospy
from std_msgs.msg import String


class NamespaceExample:
    def __init__(self):
        self.pub_global = rospy.Publisher('global_topic', String, queue_size=1)
        self.pub_private = rospy.Publisher('~private_topic', String, queue_size=1)


if __name__ == '__main__':
    rospy.init_node('namespace_example')
    node_instance = NamespaceExample()
    rospy.spin()
