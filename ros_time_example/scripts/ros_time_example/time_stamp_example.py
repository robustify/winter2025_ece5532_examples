#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, TwistStamped


class TimeStampExample:
    def __init__(self):
        self.pub_twist_stamped = rospy.Publisher('twist_stamped', TwistStamped, queue_size=1)
        self.sub_twist = rospy.Subscriber('twist', Twist, callback=self.recv_twist)
        self.start_time = rospy.Time.now()

    def recv_twist(self, msg):
        twist_stamped_msg = TwistStamped()

        twist_stamped_msg.header.stamp = rospy.Time.now()
        twist_stamped_msg.twist = msg

        self.pub_twist_stamped.publish(twist_stamped_msg)

        time_since_start = twist_stamped_msg.header.stamp - self.start_time
        rospy.loginfo(f'Received twist message {time_since_start.to_sec()} seconds from start')


if __name__ == '__main__':
    rospy.init_node('time_stamp_example')
    node_instance = TimeStampExample()
    rospy.spin()