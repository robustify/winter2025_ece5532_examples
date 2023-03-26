#pragma once

#include <ros/ros.h>
#include <mantis_model/MantisPluginConfig.h>

namespace mantis_model
{

  class MotorController
  {
    public:
      MotorController();
      void reset();
      double update(double ts, double cmd, double feedback, const mantis_model::MantisPluginConfig& params);

    private:
      double int_val_;
  };

}