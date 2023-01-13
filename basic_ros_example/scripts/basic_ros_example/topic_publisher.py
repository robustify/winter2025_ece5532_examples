#! /usr/bin/env python3
import rospy
from std_msgs.msg import String


class TopicPublisher:
    def __init__(self):
        self.sub_string = rospy.Subscriber('topic_in', String, callback=self.recv_string)
        self.pub_string = rospy.Publisher('topic_out', String, queue_size=1)

    def recv_string(self, msg):
        new_string_msg = String()
        new_string_msg.data = msg.data + '_123'
        self.pub_string.publish(new_string_msg)


if __name__ == '__main__':
    rospy.init_node('topic_publisher')
    node_instance = TopicPublisher()
    rospy.spin()