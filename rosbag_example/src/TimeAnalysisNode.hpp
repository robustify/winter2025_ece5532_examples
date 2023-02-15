#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

// Namespace matches ROS package name
namespace rosbag_example {

  class TimeAnalysisNode {
    public:
      TimeAnalysisNode(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
      
      ros::Timer timer_;
      ros::Subscriber sub_twist_;

  };

}
