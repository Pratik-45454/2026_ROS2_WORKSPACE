#include <stdio.h>

#include "ServoMotor.hpp"
#include "maths/math.hpp"

bool ServoMotor::init()
{
  bool is_success = true;
  is_success &= motor.init();
  is_success &= encoder.init();
  pid.init();
  return is_success;
}

bool ServoMotor::update()
{
  encoder.get_omega();
  pose = get_encoder_pose_radians();
  pid.setpoint = setpoint;
  pid.input=pose;
  pid.compute();
  output = (pid.output / max_motor_omega) * max_motor_pwm;
  motor.set_speed(output);
  return pid.is_allowable_error;
}

void ServoMotor::set_pose_radians(const float _setpoint)
{
  setpoint = _setpoint;

  if (setpoint < min_pose)
  {
    setpoint = min_pose;
  }

  if (setpoint > max_pose)
  {
    setpoint = max_pose;
  }
}

void ServoMotor::set_pose_degrees(const float _setpoint)
{
  setpoint = _setpoint * F32_PI / 180.0f;

  if (setpoint < min_pose)
  {
    setpoint = min_pose;
  }

  if (setpoint > max_pose)
  {
    setpoint = max_pose;
  }
}

void ServoMotor::reset_pose(void)
{
  set_speed(0.0f);
  pid.setpoint = 0.0f;
  pid.input = 0.0f;
  pid.output = 0.0f;
  encoder.reset_encoder_count();
  encoder.reset_count_aggregate();
}

void ServoMotor::reset_pose(float _pose)
{
  pose = _pose;
  encoder.reset_encoder_count();
  encoder.set_count_aggregate(pose / (2 * F32_PI) / gear_ratio * encoder.get_calibrated_cpr());
}
