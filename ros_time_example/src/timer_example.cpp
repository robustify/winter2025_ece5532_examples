#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer_example");
  ros::NodeHandle node;
  
  ros::spin();
}