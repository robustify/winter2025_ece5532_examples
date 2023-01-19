#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


class DiffDriveKinematics:
    def __init__(self):
        self.sub_twist = rospy.Subscriber('cmd_vel', Twist, callback=self.recv_twist)
        self.pub_left = rospy.Publisher('left_speed', Float64, queue_size=1)
        self.pub_right = rospy.Publisher('right_speed', Float64, queue_size=1)

    def recv_twist(self, msg):
        rw = 0.3
        W = 1.2

        v = msg.linear.x
        pdot = msg.angular.z

        left_wheel_command = Float64()
        right_wheel_command = Float64()

        left_wheel_command.data = (v - W * pdot / 2) / rw
        right_wheel_command.data = (v + W * pdot / 2) / rw

        self.pub_left.publish(left_wheel_command)
        self.pub_right.publish(right_wheel_command)


if __name__ == '__main__':
    rospy.init_node('diff_drive_kinematics')
    node_instance = DiffDriveKinematics()
    rospy.spin()