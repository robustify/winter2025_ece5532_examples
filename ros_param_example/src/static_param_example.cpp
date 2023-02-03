#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "static_param_example");
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  double p1;
  node.param("p1", p1, 5.0);
  ROS_INFO_STREAM("p1: " << p1);

  double p2;
  private_node.param("p2", p2, 8.0);
  ROS_INFO_STREAM("p2: " << p2);

  double p3;
  bool found_p3 = node.getParam("p3", p3);
  if (found_p3) {
    ROS_INFO_STREAM("p3: " << p3);
  } else { 
    ROS_WARN("Could not find parameter p3");
  }
  
  return 0;
}