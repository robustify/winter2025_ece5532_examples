#include <ros/ros.h>
#include <basic_ros_example/Adder.h>

bool srvCallback(basic_ros_example::Adder::Request& req, basic_ros_example::Adder::Response& res) {
  res.result = req.val1 + req.val2;
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "service_advertiser");
  ros::NodeHandle node;

  ros::ServiceServer srv = node.advertiseService("adder_service", srvCallback);

  ros::spin();
}