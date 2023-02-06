#! /usr/bin/env python3
import math
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from dynamic_reconfigure.server import Server
from marker_example.cfg import MarkerExampleConfig


class TfBroadcaster:
    def __init__(self):
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.timer = rospy.Timer(rospy.Duration(0.02), self.timer_callback)
        self.transform_msg = TransformStamped()
        self.transform_msg.header.frame_id = 'map'
        self.transform_msg.child_frame_id = 'frame1'

        self.reconfig_srv = Server(MarkerExampleConfig, self.reconfig)
    
    def reconfig(self, config, level):
        self.transform_msg.transform.translation.x = config.x
        self.transform_msg.transform.translation.y = config.y
        self.transform_msg.transform.translation.z = 0

        self.transform_msg.transform.rotation.w = math.cos(0.5 * config.yaw)
        self.transform_msg.transform.rotation.z = math.sin(0.5 * config.yaw)
        return config

    def timer_callback(self, event):
        self.transform_msg.header.stamp = event.current_real
        self.broadcaster.sendTransform(self.transform_msg)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    node_instance = TfBroadcaster()
    rospy.spin()
