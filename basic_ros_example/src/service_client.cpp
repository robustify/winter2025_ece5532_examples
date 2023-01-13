#include <ros/ros.h>
#include <basic_ros_example/Adder.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "service_client");
  ros::NodeHandle node;

  ros::ServiceClient adder_srv = node.serviceClient<basic_ros_example::Adder>("adder_service");

  basic_ros_example::AdderRequest request;
  basic_ros_example::AdderResponse response;
  request.val1 = 4.5;
  request.val2 = 1.0;

  adder_srv.waitForExistence();
  bool success = adder_srv.call(request, response);

  if (success) {
    ROS_INFO_STREAM("Service succeeded! Result: " << response.result);
  } else {
    ROS_WARN("Service call failed!");
  }

  return 0;
}