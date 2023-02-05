#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gps_example");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  ros::spin();
}
