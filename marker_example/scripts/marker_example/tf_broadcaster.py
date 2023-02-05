#! /usr/bin/env python3
import rospy


class TfBroadcaster:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    node_instance = TfBroadcaster()
    rospy.spin()
