#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "marker_example");
  ros::NodeHandle node;

  ros::spin();
}