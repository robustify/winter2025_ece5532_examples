#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>
#include <gps_common/conversions.h>

Eigen::Vector2d ref_utm;
ros::Publisher pub_path;
nav_msgs::Path path_msg;

void recvFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
  Eigen::Vector2d current_utm;
  std::string utm_zone;
  gps_common::LLtoUTM(msg->latitude, msg->longitude, current_utm.y(), current_utm.x(), utm_zone);

  Eigen::Vector2d relative_position = current_utm - ref_utm;

  geometry_msgs::PoseStamped new_path_point;
  new_path_point.pose.orientation.w = 1;
  new_path_point.pose.position.x = relative_position.x();
  new_path_point.pose.position.y = relative_position.y();
  path_msg.poses.push_back(new_path_point);

  path_msg.header.stamp = ros::Time::now();
  pub_path.publish(path_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::Subscriber sub_fix = n.subscribe("audibot/gps/fix", 1, recvFix);
  pub_path = n.advertise<nav_msgs::Path>("gps_path", 1);

  path_msg.header.frame_id = "world";

  double ref_lat;
  double ref_lon;
  bool found_param = true;
  found_param &= pn.getParam("ref_lat", ref_lat);
  found_param &= pn.getParam("ref_lon", ref_lon);
  if (!found_param) {
    ROS_ERROR("Could not find the reference coordinate parameters!");
    return 1;
  }

  std::string utm_zone;
  gps_common::LLtoUTM(ref_lat, ref_lon, ref_utm.y(), ref_utm.x(), utm_zone);

  ros::spin();
}
