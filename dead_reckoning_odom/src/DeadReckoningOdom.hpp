#pragma once

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

// Namespace matches ROS package name
namespace dead_reckoning_odom {

  class DeadReckoningOdom {
    public:
      DeadReckoningOdom(ros::NodeHandle n, ros::NodeHandle pn);

    private:
      void timerCallback(const ros::TimerEvent& event);
      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);

      ros::Subscriber sub_twist;
      ros::Timer timer;

      geometry_msgs::TwistStamped twist_data;

      tf2_ros::TransformBroadcaster broadcaster;

      double x;
      double y;
      double psi;
      double sample_time;

      std::string parent_frame;
      std::string child_frame;
  };

}
