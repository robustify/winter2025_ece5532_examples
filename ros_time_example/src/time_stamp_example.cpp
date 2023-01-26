#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "time_stamp_example");
  ros::NodeHandle node;
  
  ros::spin();
}