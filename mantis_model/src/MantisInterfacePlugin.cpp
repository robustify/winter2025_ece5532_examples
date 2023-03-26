#include "MantisInterfacePlugin.hpp"

namespace gazebo {

MantisInterfacePlugin::MantisInterfacePlugin()
{
  left_cmd_speed_ = 0;
  right_cmd_speed_ = 0;
}

void MantisInterfacePlugin::Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
  // Gazebo initialization
  left_wheel_joint_ = model->GetJoint("left_wheel_joint");
  right_wheel_joint_ = model->GetJoint("right_wheel_joint");
  footprint_link_ = model->GetLink("base_footprint");
  model_ = model;
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MantisInterfacePlugin::OnUpdate, this, _1));

  // ROS initialization
  n_ = std::make_shared<ros::NodeHandle>(model->GetName());
  srv_ = std::make_shared<dynamic_reconfigure::Server<mantis_model::MantisPluginConfig> >(*n_);
  srv_->setCallback(boost::bind(&MantisInterfacePlugin::reconfig, this, _1, _2));

  sub_left_cmd_ = n_->subscribe("left_speed_cmd", 1, &MantisInterfacePlugin::recvLeftCmd, this);
  sub_right_cmd_ = n_->subscribe("right_speed_cmd", 1, &MantisInterfacePlugin::recvRightCmd, this);
  pub_twist_ = n_->advertise<geometry_msgs::TwistStamped>("twist", 1);
  pub_left_speed_ = n_->advertise<std_msgs::Float64>("left_wheel_speed", 1);
  pub_right_speed_ = n_->advertise<std_msgs::Float64>("right_wheel_speed", 1);
  broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  timer_ = n_->createTimer(ros::Duration(0.01), &MantisInterfacePlugin::timerCallback, this);
}

void MantisInterfacePlugin::reconfig(mantis_model::MantisPluginConfig& config, uint32_t level)
{
  cfg_ = config;
  left_controller_.reset();
  right_controller_.reset();
}

void MantisInterfacePlugin::OnUpdate(const common::UpdateInfo& info) {
  ros::Time current_ros_time = ros::Time::now();
  double current_gazebo_update_time = info.simTime.Double();
  double dt = current_gazebo_update_time - last_gazebo_update_time_;
  last_gazebo_update_time_ = current_gazebo_update_time;
  if (std::abs(dt) > 1.0) {
    // Skip the first update call so dt is accurate
    return;
  }

  left_motor_.update(dt, left_voltage_output_, left_wheel_joint_);
  right_motor_.update(dt, right_voltage_output_, right_wheel_joint_);
}

void MantisInterfacePlugin::recvLeftCmd(const std_msgs::Float64ConstPtr& msg)
{
  left_cmd_speed_ = msg->data;
  left_cmd_stamp_ = ros::Time::now();
}

void MantisInterfacePlugin::recvRightCmd(const std_msgs::Float64ConstPtr& msg)
{
  right_cmd_speed_ = msg->data;
  right_cmd_stamp_ = ros::Time::now();
}

void MantisInterfacePlugin::timerCallback(const ros::TimerEvent& event)
{
  if (event.last_real == ros::Time(0)) {
    return;
  }
  double dt = (event.current_real - event.last_real).toSec();
  double actual_left_speed = left_wheel_joint_->GetVelocity(0);
  double actual_right_speed = right_wheel_joint_->GetVelocity(0);

  double left_speed_input = left_cmd_speed_;
  double right_speed_input = right_cmd_speed_;
  if ((event.current_real - left_cmd_stamp_).toSec() >= 0.25) {
    // Timeout!
    left_speed_input = 0;
  }

  if ((event.current_real - right_cmd_stamp_).toSec() >= 0.25) {
    // Timeout!
    right_speed_input = 0;
  }

  left_voltage_output_ = left_controller_.update(dt, left_speed_input, actual_left_speed, cfg_);
  right_voltage_output_ = right_controller_.update(dt, right_speed_input, actual_right_speed, cfg_);

  auto ang_vel = footprint_link_->RelativeAngularVel();
  auto lin_vel = footprint_link_->RelativeLinearVel();

  // Publish twist feedback
  geometry_msgs::TwistStamped twist_msg;
  twist_msg.header.frame_id = "base_footprint";
  twist_msg.header.stamp = event.current_real;
  twist_msg.twist.linear.x = lin_vel.X();
  twist_msg.twist.linear.y = lin_vel.Y();
  twist_msg.twist.linear.z = lin_vel.Z();
  twist_msg.twist.angular.x = ang_vel.X();
  twist_msg.twist.angular.y = ang_vel.Y();
  twist_msg.twist.angular.z = ang_vel.Z();
  pub_twist_.publish(twist_msg);

  // Publish measured wheel speeds
  std_msgs::Float64 wheel_speed_msg;
  wheel_speed_msg.data = actual_left_speed;
  pub_left_speed_.publish(wheel_speed_msg);
  wheel_speed_msg.data = actual_right_speed;
  pub_right_speed_.publish(wheel_speed_msg);

  // Broadcast ground truth TF transform
  if (cfg_.ground_truth) {
    geometry_msgs::TransformStamped pose;
    pose.header.frame_id = "world";
    pose.header.stamp = event.current_real;
    pose.child_frame_id = footprint_link_->GetName();
    pose.transform.translation.x = footprint_link_->WorldPose().Pos().X();
    pose.transform.translation.y = footprint_link_->WorldPose().Pos().Y();
    pose.transform.translation.z = footprint_link_->WorldPose().Pos().Z();
    pose.transform.rotation.x = footprint_link_->WorldPose().Rot().X();
    pose.transform.rotation.y = footprint_link_->WorldPose().Rot().Y();
    pose.transform.rotation.z = footprint_link_->WorldPose().Rot().Z();
    pose.transform.rotation.w = footprint_link_->WorldPose().Rot().W();
    broadcaster_->sendTransform(pose);
  }
}

void MantisInterfacePlugin::Reset()
{
  left_controller_.reset();
  right_controller_.reset();
  left_motor_.reset();
  right_motor_.reset();
  left_voltage_output_ = 0.0;
  right_voltage_output_ = 0.0;
}

MantisInterfacePlugin::~MantisInterfacePlugin()
{
  n_->shutdown();
}

}