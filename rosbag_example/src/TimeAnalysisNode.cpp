// Header file for the class
#include "TimeAnalysisNode.hpp"

// Namespace matches ROS package name
namespace rosbag_example {

  // Constructor with global and private node handle arguments
  TimeAnalysisNode::TimeAnalysisNode(ros::NodeHandle& n, ros::NodeHandle& pn)
  {
    // Create a timer to run at 1 Hz and call the 'timerCallback' method every trigger
    timer_ = n.createTimer(ros::Duration(1.0), &TimeAnalysisNode::timerCallback, this);

    // Subscribe to twist topic coming from bag file
    sub_twist_ = n.subscribe("twist", 1, &TimeAnalysisNode::recvTwist, this);
  }

  void TimeAnalysisNode::timerCallback(const ros::TimerEvent& event)
  {
    ROS_INFO("Current time stamp in seconds: %f\n", event.current_real.toSec());
  }

  void TimeAnalysisNode::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg)
  {
    ROS_INFO("Received message with stamp %f", msg->header.stamp.toSec());

    ros::Duration message_age = ros::Time::now() - msg->header.stamp;
    ROS_INFO("Message stamp is %f seconds old\n", message_age.toSec());
  }
}
