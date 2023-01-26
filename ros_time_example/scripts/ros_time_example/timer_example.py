#! /usr/bin/env python3
import rospy


class TimerExample:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('timer_example')
    node_instance = TimerExample()
    rospy.spin()