#! /usr/bin/env python3
import rospy
from visualization_msgs.msg import MarkerArray, Marker


class MarkerExample:
    def __init__(self):
        self.pub_marker_array = rospy.Publisher('marker_array', MarkerArray, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.timer_callback)

        # Cube marker
        self.cube_marker_msg = Marker()
        self.cube_marker_msg.header.frame_id = "map"
        self.cube_marker_msg.id = 0
        self.cube_marker_msg.action = Marker.ADD
        self.cube_marker_msg.type = Marker.CUBE

        self.cube_marker_msg.pose.position.x = 5.0
        self.cube_marker_msg.pose.position.y = 6.0
        self.cube_marker_msg.pose.position.z = 0.0
        self.cube_marker_msg.pose.orientation.w = 1.0

        self.cube_marker_msg.scale.x = 1.0
        self.cube_marker_msg.scale.y = 1.0
        self.cube_marker_msg.scale.z = 1.0

        self.cube_marker_msg.color.a = 1.0
        self.cube_marker_msg.color.r = 1.0
        self.cube_marker_msg.color.g = 1.0
        self.cube_marker_msg.color.b = 0.0

        # Arrow marker
        self.arrow_marker_msg = Marker()
        self.arrow_marker_msg.header.frame_id = "frame1"
        self.arrow_marker_msg.id = 1
        self.arrow_marker_msg.action = Marker.ADD
        self.arrow_marker_msg.type = Marker.ARROW

        self.arrow_marker_msg.pose.position.x = -5.0
        self.arrow_marker_msg.pose.position.y = 6.0
        self.arrow_marker_msg.pose.position.z = 0.0
        self.arrow_marker_msg.pose.orientation.w = 1.0

        self.arrow_marker_msg.scale.x = 2.0
        self.arrow_marker_msg.scale.y = 0.1
        self.arrow_marker_msg.scale.z = 0.1

        self.arrow_marker_msg.color.a = 1.0
        self.arrow_marker_msg.color.r = 1.0
        self.arrow_marker_msg.color.g = 0.0
        self.arrow_marker_msg.color.b = 1.0

    def timer_callback(self, event):
        marker_array_msg = MarkerArray()

        self.cube_marker_msg.header.stamp = event.current_real
        self.arrow_marker_msg.header.stamp = event.current_real
        marker_array_msg.markers.append(self.cube_marker_msg)
        marker_array_msg.markers.append(self.arrow_marker_msg)
        self.pub_marker_array.publish(marker_array_msg)


if __name__ == '__main__':
    rospy.init_node('marker_example')
    node_instance = MarkerExample()
    rospy.spin()
