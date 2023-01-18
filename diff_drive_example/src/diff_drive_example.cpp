#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "diff_drive_node");
  ros::NodeHandle n;

  ros::spin();
}
