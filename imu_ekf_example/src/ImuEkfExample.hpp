#pragma once

#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <imu_ekf_example/ImuEkfExampleConfig.h>

#include <eigen3/Eigen/Dense>

namespace imu_ekf_example {

  typedef Eigen::Matrix<double, 7, 1> StateVector;
  typedef Eigen::Matrix<double, 7, 7> StateMatrix;
  enum { QW=0, QX, QY, QZ, RATE_X, RATE_Y, RATE_Z };

  class ImuEkfExample {
    public:
      ImuEkfExample(ros::NodeHandle n, ros::NodeHandle pn);

    private:
      void reconfig(ImuEkfExampleConfig& config, uint32_t level);
      void recvImu(const sensor_msgs::ImuConstPtr& msg);

      // Methods to predict states and propagate uncertainty 
      StateVector statePrediction(double dt, const StateVector& old_state);
      StateMatrix stateJacobian(double dt, const StateVector& state);
      StateMatrix covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov);

      ros::Subscriber sub_imu_;

      dynamic_reconfigure::Server<ImuEkfExampleConfig> srv_;

      tf2_ros::TransformBroadcaster broadcaster_;

      // Estimate state, covariance, and current time stamp
      StateVector X_;
      StateMatrix P_;
      ros::Time estimate_stamp_;

      // Process noise covariance
      StateMatrix Q_;

      // Measurement noise covariance
      Eigen::Matrix<double, 6, 6> R_;
  };
}
