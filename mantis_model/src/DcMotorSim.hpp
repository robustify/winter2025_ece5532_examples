#pragma once
#include <ros/ros.h>
#include <gazebo/physics/physics.hh>

namespace mantis_model {

  class DcMotorSim {
    public:
      DcMotorSim();
      void reset();
      void update (double ts, double voltage_in, const gazebo::physics::JointPtr& joint);
    
    private:
      // Electrical properties
      double current_;
      double inductance_;
      double resistance_;
      double torque_constant_;
      double stall_current_;

      // Mechanical properties
      double viscous_dampening_factor_;
      double rolling_resistance_torque_;
  };

}