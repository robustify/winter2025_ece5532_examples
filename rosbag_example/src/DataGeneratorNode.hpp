#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

// Namespace matches ROS package name
namespace rosbag_example {

  class DataGeneratorNode {
    public:
      DataGeneratorNode(ros::NodeHandle& n, ros::NodeHandle& pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      
      ros::Timer timer_;
      ros::Publisher pub_twist_;

      double sine_wave_time;

  };

}
