// ROS and node class header file
#include <ros/ros.h>
#include "ImuEkfExample.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "imu_ekf_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  // Instantiate node class
  imu_ekf_example::ImuEkfExample node(n, pn);

  // Spin and process callbacks
  ros::spin();
}
