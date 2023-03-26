#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <mantis_model/MantisPluginConfig.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include "DcMotorSim.hpp"
#include "MotorController.hpp"

namespace gazebo {

  class MantisInterfacePlugin : public ModelPlugin
  {
    public:
      MantisInterfacePlugin();
      virtual ~MantisInterfacePlugin();

    protected:
      virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
      virtual void Reset();

    private:

      // ROS interfaces
      void timerCallback(const ros::TimerEvent& event);
      void recvLeftCmd(const std_msgs::Float64ConstPtr& msg);
      void recvRightCmd(const std_msgs::Float64ConstPtr& msg);
      void reconfig(mantis_model::MantisPluginConfig& config, uint32_t level);
      std::shared_ptr<ros::NodeHandle> n_;
      std::shared_ptr<dynamic_reconfigure::Server<mantis_model::MantisPluginConfig> > srv_;
      mantis_model::MantisPluginConfig cfg_;
      ros::Publisher pub_twist_;
      ros::Publisher pub_left_speed_;
      ros::Publisher pub_right_speed_;
      ros::Subscriber sub_left_cmd_;
      ros::Subscriber sub_right_cmd_;
      ros::Timer timer_;
      std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;

      // Gazebo physics engine interfaces
      void OnUpdate(const common::UpdateInfo& info);
      physics::ModelPtr model_;
      event::ConnectionPtr update_connection_;
      physics::JointPtr left_wheel_joint_;
      physics::JointPtr right_wheel_joint_;
      physics::LinkPtr footprint_link_;

      // Persistent variables
      ros::Time left_cmd_stamp_;
      ros::Time right_cmd_stamp_;
      double left_cmd_speed_;
      double right_cmd_speed_;
      double last_gazebo_update_time_;
      double left_voltage_output_;
      double right_voltage_output_;

      // Motor dynamics class instances
      mantis_model::DcMotorSim left_motor_;
      mantis_model::DcMotorSim right_motor_;

      // PI motor controller instances
      mantis_model::MotorController left_controller_;
      mantis_model::MotorController right_controller_;
  };

  GZ_REGISTER_MODEL_PLUGIN(MantisInterfacePlugin)

}
