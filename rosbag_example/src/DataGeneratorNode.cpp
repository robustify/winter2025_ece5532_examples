// Header file for the class
#include "DataGeneratorNode.hpp"

// Namespace matches ROS package name
namespace rosbag_example {

  // Constructor with global and private node handle arguments
  DataGeneratorNode::DataGeneratorNode(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    sine_wave_time = 0.0;
    
    pub_twist_ = n.advertise<geometry_msgs::TwistStamped>("twist", 1);
    timer_ = n.createTimer(ros::Duration(0.1), &DataGeneratorNode::timerCallback, this);
  }

  void DataGeneratorNode::timerCallback(const ros::TimerEvent& event)
  {
    if (event.last_real == ros::Time(0)) {
      // Skip first timer trigger
      return;
    }

    // Compute time difference between the previous and current timer trigger
    double time_diff = (event.current_real - event.last_real).toSec();

    // Declare message structure variable
    geometry_msgs::TwistStamped twist_msg;
    twist_msg.header.stamp = event.current_real; // Stamp message at current ROS time
    
    // Generate a 20-second sine wave in forward speed field
    sine_wave_time += time_diff;
    twist_msg.twist.linear.x = 5 * (1 - cos(2 * M_PI / 20.0 * sine_wave_time));

    // Constant yaw rate
    twist_msg.twist.angular.z = 0.1;

    pub_twist_.publish(twist_msg);
  }
}
