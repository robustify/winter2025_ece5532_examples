#include "MotorController.hpp"

namespace mantis_model
{

  MotorController::MotorController()
  {
    reset();
  }

  void MotorController::reset()
  {
    int_val_ = 0.0;
  }

  double MotorController::update(double ts, double cmd, double feedback, const mantis_model::MantisPluginConfig& params)
  {
    // Compute voltage command
    double error = cmd - feedback;
    double voltage_output = params.kp * error + params.ki * int_val_;
    int_val_ += ts * error;
    return voltage_output;
  }

}