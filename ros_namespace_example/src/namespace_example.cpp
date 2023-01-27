#include <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "namespace_example");

  ros::NodeHandle global_handle;
  ros::NodeHandle private_handle("~");

  ros::Publisher pub_global_topic = global_handle.advertise<std_msgs::String>("global_topic", 1);

  ros::Publisher pub_private_topic = private_handle.advertise<std_msgs::String>("private_topic", 1);

  ros::Publisher pub_override_topic = private_handle.advertise<std_msgs::String>("/another_topic", 1);
  
  ros::spin();
}