#pragma once

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <gps_common/conversions.h>
#include <dynamic_reconfigure/server.h>
#include <gnss_ekf_example/GnssEkfExampleConfig.h>

#include <eigen3/Eigen/Dense>

namespace gnss_ekf_example {

  typedef Eigen::Matrix<double, 5, 1> StateVector;
  typedef Eigen::Matrix<double, 5, 5> StateMatrix;
  enum { POS_X=0, POS_Y, HEADING, SPEED, YAW_RATE };

  class GnssEkfExample {
    public:
      GnssEkfExample(ros::NodeHandle n, ros::NodeHandle pn);

    private:
      void reconfig(GnssEkfExampleConfig& config, uint32_t level);
      void recvTwist(const geometry_msgs::TwistStampedConstPtr& msg);
      void recvFix(const sensor_msgs::NavSatFixConstPtr& msg);

      // Methods to iterate the Kalman filter
      void updateFilterGPS(const ros::Time& current_time, const Eigen::Vector2d& position);
      void updateFilterTwist(const ros::Time& current_time, const geometry_msgs::Twist& twist);

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      ros::Subscriber sub_twist_;
      ros::Subscriber sub_fix_;

      dynamic_reconfigure::Server<GnssEkfExampleConfig> srv_;
      GnssEkfExampleConfig cfg_;

      tf2_ros::TransformBroadcaster broadcaster_;
      Eigen::Vector2d ref_utm_;

      // Estimate state, covariance, and current time stamp
      StateVector X_;
      StateMatrix P_;
      ros::Time estimate_stamp_;

      // Process noise covariance
      StateMatrix Q_;
  };

}
