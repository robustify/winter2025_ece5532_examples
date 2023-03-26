#include "DcMotorSim.hpp"

namespace mantis_model {

  DcMotorSim::DcMotorSim()
  {
    // Electrical properties
    inductance_ = 0.0025;
    resistance_ = 0.5;
    torque_constant_ = 0.35;
    stall_current_ = 30.0;
    current_ = 0.0;

    // Mechanical properties
    viscous_dampening_factor_ = 0.3;
    rolling_resistance_torque_ = 1.0;
  }

  void DcMotorSim::reset()
  {
    current_ = 0.0;
  }

  void DcMotorSim::update(double ts, double voltage_in, const gazebo::physics::JointPtr& joint)
  {
    double latest_motor_velocity = joint->GetVelocity(0);

    // Iterate the difference equation for current
    current_ = (1 - ts * resistance_ / inductance_) * current_
               + ts * (voltage_in - torque_constant_ * latest_motor_velocity) / inductance_;
    if (current_ > stall_current_) {
      current_ = stall_current_;
    } else if (current_ < -stall_current_) {
      current_ = -stall_current_;
    }

    // Apply motor torque
    joint->SetForce(0, torque_constant_ * current_);
    
    // Apply dampening torque
    joint->SetForce(0, -viscous_dampening_factor_ * latest_motor_velocity);
    
    // Apply rolling resistance torque
    if (latest_motor_velocity > 0.05) {
      joint->SetForce(0, -rolling_resistance_torque_);
    } else if (latest_motor_velocity < -0.05) {
      joint->SetForce(0, rolling_resistance_torque_);
    } else {
      joint->SetForce(0, -2 * rolling_resistance_torque_ / 0.1 * latest_motor_velocity);
    }
  }

}
