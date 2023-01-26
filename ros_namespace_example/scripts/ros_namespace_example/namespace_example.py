#! /usr/bin/env python3
import rospy


class NamespaceExample:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('namespace_example')
    node_instance = NamespaceExample()
    rospy.spin()
