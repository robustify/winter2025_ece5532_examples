// ROS and node class header file
#include <ros/ros.h>
#include "DataGeneratorNode.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS and declare node handles
  ros::init(argc, argv, "data_generator_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  
  // Instantiate node class
  rosbag_example::DataGeneratorNode node(n, pn);
  
  // Spin and process callbacks
  ros::spin();
}
