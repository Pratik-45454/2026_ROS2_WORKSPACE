
#ifndef OMNIWHEEL_HPP_
#define OMNIWHEEL_HPP_

//#include "arm_math.h"
#include "math.h"
#include "../common/processor_type.h"
#include "../msgs/state_msg.hpp"

class OmniwheelKinematics
{
private:
  float base_diameter, wheel_diameter;
  float motor_offsets[4] = {M_PI_4, 3 * M_PI_4, -3 * M_PI_4, -M_PI_4};
  float inverseCouplingMatrix[4][3];

public:
  OmniwheelKinematics(const float _base_diameter, const float _wheel_diameter)
      : base_diameter(_base_diameter), wheel_diameter(_wheel_diameter)
  {
    for (int i = 0; i < 4; i++)
    {
      // inverseCouplingMatrix[i][0] = arm_sin_f32(motor_offsets[i]);
      // inverseCouplingMatrix[i][1] = -arm_cos_f32(motor_offsets[i]);
      // inverseCouplingMatrix[i][2] = -base_diameter / 2.0f;

      // We are only using this once, so it should be fine
      inverseCouplingMatrix[i][0] = sin(motor_offsets[i]);
      inverseCouplingMatrix[i][1] = -cos(motor_offsets[i]);
      inverseCouplingMatrix[i][2] = -base_diameter / 2.0f;

    }
  }

  OmniwheelKinematics(const float _base_diameter, const float _wheel_diameter, const float *_motor_offsets)
      : base_diameter(_base_diameter), wheel_diameter(_wheel_diameter),
        motor_offsets{_motor_offsets[0], _motor_offsets[1], _motor_offsets[2], _motor_offsets[3]} {}

  OmniwheelKinematics() = default;
  OmniwheelKinematics(const OmniwheelKinematics &) = default;
  ~OmniwheelKinematics() = default;

  void get_motor_omega(const Twist _twist, float *_motor_omega)
  {
    for (int i = 0; i < 4; i++)
    {
      _motor_omega[i] = inverseCouplingMatrix[i][0] * _twist.vx;
      _motor_omega[i] += inverseCouplingMatrix[i][1] * _twist.vy;
      _motor_omega[i] += inverseCouplingMatrix[i][2] * _twist.w;
      _motor_omega[i] *= 2.0f / wheel_diameter;
    }
  }
};

#endif // OMNIWHEEL_HPP_
