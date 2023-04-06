// ROS and node class header file
#include <ros/ros.h>
#include "GnssEkfExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "gnss_ekf_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  gnss_ekf_example::GnssEkfExample node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
