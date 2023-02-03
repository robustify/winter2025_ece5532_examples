#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub_marker_array;
visualization_msgs::Marker cube_marker_msg;
visualization_msgs::Marker arrow_marker_msg;

void timerCallback(const ros::TimerEvent& event)
{
  visualization_msgs::MarkerArray marker_array_msg;

  cube_marker_msg.header.stamp = event.current_real;
  arrow_marker_msg.header.stamp = event.current_real;
  marker_array_msg.markers.push_back(cube_marker_msg);
  marker_array_msg.markers.push_back(arrow_marker_msg);

  pub_marker_array.publish(marker_array_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_example");
  ros::NodeHandle node;

  pub_marker_array = node.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

  ros::Timer marker_pub_timer = node.createTimer(ros::Duration(0.05), timerCallback);

  // Cube marker
  cube_marker_msg.header.frame_id = "map";
  cube_marker_msg.id = 0;
  cube_marker_msg.action = visualization_msgs::Marker::ADD;
  cube_marker_msg.type = visualization_msgs::Marker::CUBE;

  cube_marker_msg.pose.position.x = 5.0;
  cube_marker_msg.pose.position.y = 6.0;
  cube_marker_msg.pose.position.z = 0.0;
  cube_marker_msg.pose.orientation.w = 1.0;

  cube_marker_msg.scale.x = 1.0;
  cube_marker_msg.scale.y = 1.0;
  cube_marker_msg.scale.z = 1.0;

  cube_marker_msg.color.a = 1.0;
  cube_marker_msg.color.r = 1.0;
  cube_marker_msg.color.g = 1.0;
  cube_marker_msg.color.b = 0.0;

  // Arrow marker
  arrow_marker_msg.header.frame_id = "frame1";
  arrow_marker_msg.id = 1;
  arrow_marker_msg.action = visualization_msgs::Marker::ADD;
  arrow_marker_msg.type = visualization_msgs::Marker::ARROW;

  arrow_marker_msg.pose.position.x = -5.0;
  arrow_marker_msg.pose.position.y = 6.0;
  arrow_marker_msg.pose.position.z = 0.0;
  arrow_marker_msg.pose.orientation.w = 1.0;

  arrow_marker_msg.scale.x = 2.0;
  arrow_marker_msg.scale.y = 0.1;
  arrow_marker_msg.scale.z = 0.1;

  arrow_marker_msg.color.a = 1.0;
  arrow_marker_msg.color.r = 1.0;
  arrow_marker_msg.color.g = 0.0;
  arrow_marker_msg.color.b = 1.0;

  ros::spin();
}