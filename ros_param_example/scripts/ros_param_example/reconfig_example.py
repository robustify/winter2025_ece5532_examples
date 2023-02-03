#! /usr/bin/env python3
import rospy


class ReconfigExample:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('reconfig_example')
    node_instance = ReconfigExample()
    rospy.spin()
