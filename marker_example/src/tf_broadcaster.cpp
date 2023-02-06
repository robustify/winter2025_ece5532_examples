#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <marker_example/MarkerExampleConfig.h>

geometry_msgs::TransformStamped transform_msg;
std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster;

void timerCallback(const ros::TimerEvent& event)
{
  transform_msg.header.stamp = event.current_real;
  broadcaster->sendTransform(transform_msg);
}

void reconfig(marker_example::MarkerExampleConfig& config, uint32_t level)
{
  transform_msg.transform.translation.x = config.x;
  transform_msg.transform.translation.y = config.y;
  transform_msg.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0, 0, config.yaw);
  tf2::convert(q, transform_msg.transform.rotation);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node;

  broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>();
  transform_msg.header.frame_id = "map";
  transform_msg.child_frame_id = "frame1";

  ros::Timer timer = node.createTimer(ros::Duration(0.02), timerCallback);

  dynamic_reconfigure::Server<marker_example::MarkerExampleConfig> srv;
  srv.setCallback(boost::bind(reconfig, _1, _2));

  ros::spin();
}