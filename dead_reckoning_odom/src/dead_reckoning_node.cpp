// ROS and node class header file
#include <ros/ros.h>
#include "DeadReckoningOdom.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "dead_reckoning");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  dead_reckoning_odom::DeadReckoningOdom node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
