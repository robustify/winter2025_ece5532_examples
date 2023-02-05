#! /usr/bin/env python3
import rospy


class GpsExample:
    def __init__(self):
        pass

    
if __name__ == '__main__':
    rospy.init_node('gps_example')
    node_instance = GpsExample()
    rospy.spin()