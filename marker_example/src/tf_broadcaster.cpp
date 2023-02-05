#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;

  ros::spin();
}