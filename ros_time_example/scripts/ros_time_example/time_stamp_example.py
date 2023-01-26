#! /usr/bin/env python3
import rospy


class TimeStampExample:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('time_stamp_example')
    node_instance = TimeStampExample()
    rospy.spin()