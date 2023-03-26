#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState


class ExampleJointPublisher:
    def __init__(self):
        self.update_period = 0.02

        self.pub_joints = rospy.Publisher('joint_states', JointState, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(self.update_period), self.timer_callback)

        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['joint1']
        self.joint_state_msg.position = [0.0]
        self.joint_state_msg.velocity = [0.0]
        self.joint_state_msg.effort = [0.0]

    def timer_callback(self, event):
        speed = 1.0
        self.joint_state_msg.header.stamp = event.current_real
        self.joint_state_msg.position[0] += self.update_period * speed
        self.joint_state_msg.velocity[0] = speed
        self.pub_joints.publish(self.joint_state_msg)


if __name__ == '__main__':
    rospy.init_node('example_joint_publisher')
    node_instance = ExampleJointPublisher()
    rospy.spin()
