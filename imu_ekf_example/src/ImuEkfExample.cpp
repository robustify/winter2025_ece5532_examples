#include "ImuEkfExample.hpp"

namespace imu_ekf_example {

  ImuEkfExample::ImuEkfExample(ros::NodeHandle n, ros::NodeHandle pn)
  {
    // Subscribe to accelerometer and gyro sensor data
    sub_imu_ = n.subscribe("imu", 1, &ImuEkfExample::recvImu, this);

    // Set up dynamic reconfigure server
    srv_.setCallback(boost::bind(&ImuEkfExample::reconfig, this, _1, _2));

    // Initialize Kalman filter state
    X_ << 1.0, 0, 0, 0, 0, 0, 0;
    P_ = 0.01 * StateMatrix::Identity();
  }

  void ImuEkfExample::recvImu(const sensor_msgs::ImuConstPtr& msg)
  {
    // Initialize state estimate directly if this is the first measurement
    if (estimate_stamp_ == ros::Time(0)) {
      X_ << 1.0, 0, 0, 0, 0, 0, 0;
      P_ = 0.01 * StateMatrix::Identity();
      estimate_stamp_ = msg->header.stamp;
      return;
    }

    // Compute amount of time to advance the state prediction
    double dt = (msg->header.stamp - estimate_stamp_).toSec();

    // Propagate estimate prediction and store in predicted variables
    StateMatrix A = stateJacobian(dt, X_);
    StateVector predicted_state = statePrediction(dt, X_);
    StateMatrix predicted_cov = covPrediction(A, Q_, P_);

    // Construct C matrix for an IMU update (3D accelerometer and gyro measurements)
    Eigen::Matrix<double, 6, 7> C;
    C.row(0) << -2 * predicted_state(QY), 2 * predicted_state(QZ), -2 * predicted_state(QW), 2  * predicted_state(QX), 0, 0, 0;
    C.row(1) << 2 * predicted_state(QX), 2 * predicted_state(QW), 2 * predicted_state(QZ), 2 * predicted_state(QY), 0, 0, 0;
    C.row(2) << 2 * predicted_state(QW), -2 * predicted_state(QX), -2 * predicted_state(QY), 2 * predicted_state(QZ), 0, 0, 0;
    C.row(3) << 0, 0, 0, 0, 1, 0, 0;
    C.row(4) << 0, 0, 0, 0, 0, 1, 0;
    C.row(5) << 0, 0, 0, 0, 0, 0, 1;

    // Use C and predicted state to compute expected measurement
    Eigen::Matrix<double, 6, 1> expected_meas;
    expected_meas << 2 * predicted_state(QX) * predicted_state(QZ) - 2 * predicted_state(QW) * predicted_state(QY),
                     2 * predicted_state(QW) * predicted_state(QX) + 2 * predicted_state(QY) * predicted_state(QZ),
                     predicted_state(QW) * predicted_state(QW) - predicted_state(QX) * predicted_state(QX) - predicted_state(QY) * predicted_state(QY) + predicted_state(QZ) * predicted_state(QZ),
                     predicted_state(RATE_X),
                     predicted_state(RATE_Y),
                     predicted_state(RATE_Z);

    // Put measurements in an Eigen object
    Eigen::Matrix<double, 6, 1> real_meas;
    Eigen::Vector3d accel_vect(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    accel_vect.normalize();
    real_meas << accel_vect.x(), accel_vect.y(), accel_vect.z(),
                 msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

    // Compute Kalman gain
    Eigen::Matrix<double, 6, 6> S;
    S = C * predicted_cov * C.transpose() + R_;
    Eigen::Matrix<double, 7, 6> K;
    K = predicted_cov * C.transpose() * S.inverse();

    // Update filter estimate based on difference between actual and expected measurements
    X_ = predicted_state + K * (real_meas - expected_meas);
    X_.head<4>().normalize();
    
    // Update estimate error covariance using Kalman gain matrix
    P_ = (StateMatrix::Identity() - K * C) * predicted_cov;

    estimate_stamp_ = msg->header.stamp;

    // Update TF transform with current estimate
    geometry_msgs::TransformStamped ekf_transform;
    ekf_transform.header.stamp = estimate_stamp_;
    ekf_transform.header.frame_id = "map";
    ekf_transform.child_frame_id = "filter";
    ekf_transform.transform.rotation.w = X_(QW);
    ekf_transform.transform.rotation.x = X_(QX);
    ekf_transform.transform.rotation.y = X_(QY);
    ekf_transform.transform.rotation.z = X_(QZ);
    ekf_transform.transform.translation.x = 0;
    ekf_transform.transform.translation.y = 0;
    ekf_transform.transform.translation.z = 0;
    broadcaster_.sendTransform(ekf_transform);
  }

  StateVector ImuEkfExample::statePrediction(double dt, const StateVector& old_state) {
    StateVector new_state;
    new_state(QW) = old_state(QW) + 0.5 * dt * (-old_state(QX) * old_state(RATE_X) - old_state(QY) * old_state(RATE_Y) - old_state(QZ) * old_state(RATE_Z));
    new_state(QX) = old_state(QX) + 0.5 * dt * ( old_state(QW) * old_state(RATE_X) + old_state(QZ) * old_state(RATE_Y) - old_state(QY) * old_state(RATE_Z));
    new_state(QY) = old_state(QY) + 0.5 * dt * ( old_state(QW) * old_state(RATE_Y) - old_state(QZ) * old_state(RATE_X) + old_state(QX) * old_state(RATE_Z));
    new_state(QZ) = old_state(QZ) + 0.5 * dt * ( old_state(QW) * old_state(RATE_Z) + old_state(QY) * old_state(RATE_X) - old_state(QX) * old_state(RATE_Y));
    new_state(RATE_X) = old_state(RATE_X);
    new_state(RATE_Y) = old_state(RATE_Y);
    new_state(RATE_Z) = old_state(RATE_Z);
    return new_state;
  }

  StateMatrix ImuEkfExample::stateJacobian(double dt, const StateVector& state) {
    StateMatrix A;
    A.row(QW) << 0, -state(RATE_X), -state(RATE_Y), -state(RATE_Z), -state(QX), -state(QY), -state(QZ);
    A.row(QX) << state(RATE_X), 0, -state(RATE_Z), state(RATE_Y), state(QW), state(QZ), -state(QY);
    A.row(QY) << state(RATE_Y), state(RATE_Z), 0, -state(RATE_X), -state(QZ), state(QW), state(QX);
    A.row(QZ) << state(RATE_Z), -state(RATE_Y), state(RATE_X), 0, state(QY), -state(QX), state(QW);
    A.row(RATE_X).setZero();
    A.row(RATE_Y).setZero();
    A.row(RATE_Z).setZero();
    return StateMatrix::Identity() + 0.5 * dt * A;
  }

  StateMatrix ImuEkfExample::covPrediction(const StateMatrix& A, const StateMatrix& Q, const StateMatrix& old_cov) {
    StateMatrix new_cov;
    new_cov = A * old_cov * A.transpose() + Q;
    return new_cov;
  }

  void ImuEkfExample::reconfig(ImuEkfExampleConfig& config, uint32_t level)
  {
    Q_.setZero();
    Q_(QW, QW) = config.q_quat * config.q_quat;
    Q_(QX, QX) = config.q_quat * config.q_quat;
    Q_(QY, QY) = config.q_quat * config.q_quat;
    Q_(QZ, QZ) = config.q_quat * config.q_quat;
    Q_(RATE_X, RATE_X) = config.q_rate * config.q_rate;
    Q_(RATE_Y, RATE_Y) = config.q_rate * config.q_rate;
    Q_(RATE_Z, RATE_Z) = config.q_rate * config.q_rate;

    R_.setZero();
    R_(0, 0) = config.r_accel * config.r_accel;
    R_(1, 1) = config.r_accel * config.r_accel;
    R_(2, 2) = config.r_accel * config.r_accel;
    R_(3, 3) = config.r_gyro * config.r_gyro;
    R_(4, 4) = config.r_gyro * config.r_gyro;
    R_(5, 5) = config.r_gyro * config.r_gyro;
  }

}