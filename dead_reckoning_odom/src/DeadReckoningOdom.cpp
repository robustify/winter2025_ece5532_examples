// Header file for the class
#include "DeadReckoningOdom.hpp"

// Namespace matches ROS package name
namespace dead_reckoning_odom {

  // Constructor with global and private node handle arguments
  DeadReckoningOdom::DeadReckoningOdom(ros::NodeHandle n, ros::NodeHandle pn) {
    // Load frame IDs and initial state values from ROS parameters
    pn.param("parent_frame", parent_frame, std::string("odom"));
    pn.param("child_frame", child_frame, std::string("base_footprint"));
    pn.param("initial_x", x, 0.0);
    pn.param("initial_y", y, 0.0);
    pn.param("initial_psi", psi, 0.0);

    sample_time = 0.02;
    sub_twist = n.subscribe("twist", 1, &DeadReckoningOdom::recvTwist, this);
    timer = n.createTimer(ros::Duration(sample_time), &DeadReckoningOdom::timerCallback, this);
  }

  void DeadReckoningOdom::timerCallback(const ros::TimerEvent& event) {
    // TODO: Integrate discrete vehicle navigation state space model one step with the latest speed and yaw rate data
    double v = twist_data.twist.linear.x;
    double pdot = twist_data.twist.angular.z;
    x += sample_time * v * cos(psi);
    y += sample_time * v * sin(psi);
    psi += sample_time * pdot;

    // Copy dead reckoning estimate into transform message
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.frame_id = parent_frame;
    transform_msg.header.stamp = event.current_real;
    transform_msg.child_frame_id = child_frame;
    transform_msg.transform.translation.x = x;
    transform_msg.transform.translation.y = y;
    transform_msg.transform.translation.z = 0;

    // Populate quaternion corresponding to current yaw angle
    tf2::Quaternion q;
    q.setRPY(0, 0, psi);
    tf2::convert(q, transform_msg.transform.rotation);

    // Broadcast TF transform with the latest dead reckoning iteration
    broadcaster.sendTransform(transform_msg);
  }

  void DeadReckoningOdom::recvTwist(const geometry_msgs::TwistStampedConstPtr& msg) {
    twist_data = *msg;
  }
}
