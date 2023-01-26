#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "namespace_example");
  
  ros::spin();
}