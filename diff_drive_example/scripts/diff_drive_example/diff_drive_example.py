#! /usr/bin/env python3
import rospy


class DiffDriveKinematics:
    def __init__(self):
        pass


if __name__ == '__main__':
    rospy.init_node('diff_drive_kinematics')
    node_instance = DiffDriveKinematics()
    rospy.spin()