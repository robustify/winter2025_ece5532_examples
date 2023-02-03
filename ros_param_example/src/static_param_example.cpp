#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_param_example");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");
  
  return 0;
}