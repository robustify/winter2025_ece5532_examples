#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <ros_param_example/ReconfigExampleConfig.h>


void reconfig(ros_param_example::ReconfigExampleConfig& config, uint32_t level)
{
  ROS_INFO("A parameter was changed!");

  if (config.enable) {
    ROS_INFO("Enabled");
  } else {
    ROS_INFO("Disabled");
  }

  ROS_INFO_STREAM("String value: " << config.str);
  ROS_INFO_STREAM("X value: " << config.x);
  ROS_INFO_STREAM("Y value: " << config.y);

  switch (config.list) {
    case ros_param_example::ReconfigExample_Option_1:
      ROS_INFO("Option 1 was selected");
    break;
    case ros_param_example::ReconfigExample_Option_2:
      ROS_INFO("Option 2 was selected");
    break;
    case ros_param_example::ReconfigExample_Option_3:
      ROS_INFO("Option 3 was selected");
    break;
    default:
    break;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reconfig_example");
  ros::NodeHandle node;

  dynamic_reconfigure::Server<ros_param_example::ReconfigExampleConfig> srv;
  srv.setCallback(boost::bind(reconfig, _1, _2));

  ros::spin();
}