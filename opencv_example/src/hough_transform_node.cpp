#include <ros/ros.h>
#include "HoughTransform.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hough_transform_node");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  opencv_example::HoughTransform node(n, pn);

  ros::spin();
}
