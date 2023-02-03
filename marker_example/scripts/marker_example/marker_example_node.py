#! /usr/bin/env python3
import rospy


class MarkerExample:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('marker_example')
    node_instance = MarkerExample()
    rospy.spin()
