#! /usr/bin/env python3
import sys
import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import geodesy.utm


class GpsExample:
    def __init__(self):
        self.sub_fix = rospy.Subscriber('audibot/gps/fix', NavSatFix, callback=self.recv_fix)
        self.pub_path = rospy.Publisher('gps_path', Path, queue_size=1)
        self.gps_path = Path()
        self.gps_path.header.frame_id = 'world'

        try:
            ref_lat = rospy.get_param('~ref_lat')
            ref_lon = rospy.get_param('~ref_lon')
            self.ref_utm = geodesy.utm.fromLatLong(ref_lat, ref_lon, 0)
            rospy.loginfo(f'Ref coordinates: {self.ref_utm}')
        except KeyError as e:
            rospy.logerr('Could not find reference coordinate parameters!')
            sys.exit(1)

    def recv_fix(self, msg):
        current_utm = geodesy.utm.fromLatLong(msg.latitude, msg.longitude, 0)

        new_path_point = PoseStamped()
        new_path_point.pose.orientation.w = 1
        new_path_point.pose.position.x = current_utm.easting - self.ref_utm.easting
        new_path_point.pose.position.y = current_utm.northing - self.ref_utm.northing
        self.gps_path.poses.append(new_path_point)
        self.gps_path.header.stamp = rospy.Time.now()
        self.pub_path.publish(self.gps_path)

    
if __name__ == '__main__':
    rospy.init_node('gps_example')
    node_instance = GpsExample()
    rospy.spin()