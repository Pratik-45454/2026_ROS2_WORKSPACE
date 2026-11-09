/***********************************************************************************************
 * @file cmotor.cpp
 * @brief Implementation file for the CMotor class providing control for a
 *robot.
 * @author Robotics Team, Robotics Club Pulchowk Campus
 * @date 2023
 **********************************************************************************************/

#include <memory.h>
#include <stdio.h>

#include "../robotlib/maths/math.hpp"
#include "cmotor.hpp"

inline float toYawPidSetpoint(float setpoint, float current,
                              CMotorAngualarDirectionType dir = CMADT_AUTO) {
  float angleDiff = wrapAngle(setpoint) - wrapAngle(current);
  switch (dir) {
    case CMADT_AUTO:
      if (angleDiff > F32_PI) {
        angleDiff -= F32_2_PI;
      } else if (angleDiff < -F32_PI) {
        angleDiff += F32_2_PI;
      }
      break;

    case CMADT_ANTICLOCKWISE:
      if (angleDiff < 0) {
        angleDiff += F32_2_PI;
      }
      break;

    case CMADT_CLOCKWISE:
      if (angleDiff > 0) {
        angleDiff -= F32_2_PI;
      }
      break;
  }
  return current + angleDiff;
}

CMotor::CMotor(const Motor *_motors, const Encoder *_encoders,
               const OmniwheelKinematics &_kinematics, const PID *_motor_pids,
               const float &_max_motor_omega, const float *_max_motor_pwms,
               const uint16_t &_motor_compute_time)
    : motors{_motors[0], _motors[1], _motors[2], _motors[3]},
      encoders{_encoders[0], _encoders[1], _encoders[2], _encoders[3]},
      kinematics(_kinematics),
      motor_pids{_motor_pids[0], _motor_pids[1], _motor_pids[2],
                 _motor_pids[3]},
      max_motor_omega(_max_motor_omega),
      max_motor_pwms{_max_motor_pwms[0], _max_motor_pwms[1], _max_motor_pwms[2],
                     _max_motor_pwms[3]},
      motor_compute_time(_motor_compute_time) {}

bool CMotor::init() {
  bool to_return = true;
  for (int i = 0; i < 4; ++i) {
    to_return &= motors[i].init();
    to_return &= encoders[i].init();
    motor_pids[i].init();
  }

  motor_compute_tick = HAL_GetTick();
  return to_return;
}

bool CMotor::compute() {
  uint32_t current_tick = HAL_GetTick();

  if (current_tick - motor_compute_tick <
      motor_compute_time) /* Maintain sampling time */
    return false;

  yaw_pid.input += angleChange(base_state.pose.theta, prev_yaw);
 // yaw_pid.input = wrapAngle(yaw_pid.input);

  if (is_base_pids_active) {
    if (is_yaw_pid_enabled && fabs(cmd_twist.w) < 0.0005) {
      switch (yawPidState) {
        case YAW_PID_IDLE:
          output_twist.w = 0;
          wait_count = 0;
          yawPidState = YAW_PID_WAIT;
          break;

        case YAW_PID_WAIT:
          if (fabs(base_state.twist.w) < 0.001f) {
            wait_count++;
          }
          if (wait_count >= 10) {
            yawPidState = YAW_PID_INIT;
          }
          break;

        case YAW_PID_INIT:
          yaw_pid.setpoint = yaw_pid.input;
          yaw_pid.output = output_twist.w;
          yaw_pid.outputSum = 0;
          yaw_pid.init();
          yawPidState = YAW_PID_ACTIVE;
          break;

        case YAW_PID_ACTIVE:
          if (fabs(yaw_pid.setpoint - yaw_pid.input) < 5 * DEG2RAD) {
            yaw_pid.outputSum = 0;
          }
          yaw_pid.compute();
          is_yaw_set_reached = yaw_pid.is_allowable_error;
          output_twist.w = yaw_pid.output;
          break;
      }
    } else {
      output_twist.w = cmd_twist.w;
      yawPidState = YAW_PID_IDLE;
    }

    if (is_v_pid_enabled) {
      vx_pid.input = base_state.twist.vx;
      vy_pid.input = base_state.twist.vy;
      vx_pid.setpoint = cmd_twist.vx;
      vy_pid.setpoint = cmd_twist.vy;

      vx_pid.compute();
      vy_pid.compute();
      
      output_twist.vx = vx_pid.output;
      output_twist.vy = vy_pid.output;
    } else {
      output_twist.vx = cmd_twist.vx;
      output_twist.vy = cmd_twist.vy;
    }
  } else {
    output_twist = cmd_twist;
  }

  kinematics.get_motor_omega(
      output_twist, motor_omegas); /*Get each motor omegas from output_twist*/

  for (int i = 0; i < 4; ++i) {
    // encoder_omegas[i] = encoder_filters[i](encoders[i].get_omega()); /*Set
    // input to each motor pid*/
    encoder_omegas[i] = encoders[i].get_omega(); /*Set input to each motor pid*/
    motor_pids[i].input = encoder_omegas[i];
    motor_pids[i].setpoint = motor_omegas[i]; /*Set setpoint to each motor pid*/

    if (motor_pids[i].compute()) /*Update output and set speed only if new pid
                                    output is computed*/
    {
      output_motor_omegas[i] = motor_pids[i].output;
    }
  }

  prev_yaw = base_state.pose.theta;
  motor_compute_tick = current_tick;

  return true;
}

void CMotor::computeKinematicsFromCmdTwist() {
  kinematics.get_motor_omega(
      cmd_twist, output_motor_omegas); /*Get each motor omegas from cmd_twist*/
}

void CMotor::actuate() {
  for (int i = 0; i < 4; ++i) {
    motors[i].set_speed(output_motor_omegas[i] / max_motor_omega *
                        max_motor_pwms[i]); /*Set each motor speed*/
  }
}

void CMotor::stop() {
  cmd_twist = Twist(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < 4; ++i) {
    motors[i].set_speed(0.0f);
    motor_pids[i].input = 0.0f;
    motor_pids[i].setpoint = 0.0f;
    motor_pids[i].output = 0.0f;
    motor_pids[i].outputSum = 0.0f;
  }
}
void CMotor::setBasePids(const PID *_yaw_pid, const PID *_vx_pid,
                         const PID *_vy_pid) {
  if (_yaw_pid != nullptr) {
    yaw_pid = *_yaw_pid;
    is_yaw_pid_set = true;
  }
  if (_vx_pid != nullptr) {
    vx_pid = *_vx_pid;
    is_yaw_pid_set = true;
  }
  if (_vx_pid != nullptr) {
    vy_pid = *_vy_pid;
    is_vy_pid_set = true;
  }
}

bool CMotor::enableBasePids(bool is_yaw, bool is_v) {
  if (is_yaw_pid_set) is_yaw_pid_enabled = is_yaw;
  if (is_vx_pid_set && is_vy_pid_set) is_v_pid_enabled = is_v;

  if (is_base_pids_active) {
    deactivateBasePids();
    activateBasePids();
  }

  if (is_yaw == is_yaw_pid_enabled && is_v == is_v_pid_enabled) return true;

  return false;
}

bool CMotor::activateBasePids() {
  if ((!is_yaw_pid_enabled && !is_v_pid_enabled) || is_base_pids_active)
    return false;

  is_base_pids_active = true;

  if (is_yaw_pid_enabled) {
    yawPidState = YAW_PID_IDLE;
  }

  if (is_v_pid_enabled) {
    vx_pid.outputSum = 0;
    vy_pid.outputSum = 0;
    vx_pid.init();
    vy_pid.init();
  }

  return true;
}

void CMotor::deactivateBasePids() { is_base_pids_active = false; }

void CMotor::setTwist(const Twist &_twist) { cmd_twist = _twist; }
void CMotor::setBaseOmega(float _omega) { cmd_twist.w = _omega; }

Encoder *CMotor::getEncoders() { return encoders; }

Motor *CMotor::getMotors() { return motors; }

OmniwheelKinematics &CMotor::getKinematics() { return kinematics; }

Twist CMotor::getTwist() const { return cmd_twist; }

Odometry CMotor::getState() const { return base_state; }

void CMotor::feedState(const Odometry &_state) { base_state = _state; }

float *CMotor::getEncoderOmegas() { return encoder_omegas; }

void CMotor::printSetTwist() const {
  printf("CMotor:Base Set Twist:: %f %f %f\n", cmd_twist.vx, cmd_twist.vy,
         cmd_twist.w);
}

void CMotor::printOutputTwist() const {
  printf("CMotor:Base Output Twist:: %f %f %f\n", output_twist.vx,
         output_twist.vy, output_twist.w);
}

void CMotor::printEncoderOmegas() const {
  // printf("CMotor:EncoderOmegas:: %i %i %i %i\n",
  //        (int)(encoder_omegas[0]*100),
  //        (int)(encoder_omegas[1]*100),
  //        (int)(encoder_omegas[2]*100),
  //        (int)(encoder_omegas[3]*100));

  printf("%i\t%i\t%i\t%i\n", (int)(encoder_omegas[0] * 100),
         (int)(encoder_omegas[1] * 100), (int)(encoder_omegas[2] * 100),
         (int)(encoder_omegas[3] * 100));
}

void CMotor::printMotorGraph(int n) {
  static unsigned int testtime;
  if (n > 3 || n < 0) return;

  if (HAL_GetTick() - testtime < 50) return;

  printf("%i\t%i\n", (int)(encoder_omegas[n] * 100),
         (int)(motor_omegas[n] * 100));

  testtime = HAL_GetTick();
}

void CMotor::printMotorGraphAll() {
  static unsigned int testtime;

  if (HAL_GetTick() - testtime < 50) return;

  printf("%i\t%i\t%i\t%i\t%i\t%i\t%i\t%i\n", (int)(encoder_omegas[0] * 100),
         (int)(encoder_omegas[1] * 100), (int)(encoder_omegas[2] * 100),
         (int)(encoder_omegas[3] * 100), (int)(motor_omegas[0] * 100),
         (int)(motor_omegas[1] * 100), (int)(motor_omegas[2] * 100),
         (int)(motor_omegas[3] * 100));

  testtime = HAL_GetTick();
}

void CMotor::printSetGraphAll() {
  static unsigned int testtime;

  if (HAL_GetTick() - testtime < 50) return;

  printf("%i\t%i\t%i\t%i\n", (int)(motor_omegas[0] * 100),
         (int)(motor_omegas[1] * 100), (int)(motor_omegas[2] * 100),
         (int)(motor_omegas[3] * 100));

  testtime = HAL_GetTick();
}

void CMotor::printEncoderCounts() {
  printf("CMotor:EncoderCounts:: %ld %ld %ld %ld\n",
         (int32_t)encoders[0].get_count_aggregate(),
         (int32_t)encoders[1].get_count_aggregate(),
         (int32_t)encoders[2].get_count_aggregate(),
         (int32_t)encoders[3].get_count_aggregate());
}

void CMotor::printSetpointMotorOmegas() const {
  printf("CMotor:SetMotorOmegas:: %i %i %i %i\n", (int)(motor_omegas[0] * 100),
         (int)(motor_omegas[1] * 100), (int)(motor_omegas[2] * 100),
         (int)(motor_omegas[3] * 100));
}

void CMotor::printMotorPidOutput() const {
  printf("CMotor:PidOutputs:: %i %i %i %i\n", (int)(motor_pids[0].output * 100),
         (int)(motor_pids[1].output * 100), (int)(motor_pids[2].output * 100),
         (int)(motor_pids[3].output * 100));
}

void CMotor::printBaseState() const {
  printf("CMotor:: base_sate :: x:%f y:%f theta:%f vx:%f vy:%f w:%f\n",
         base_state.pose.x, base_state.pose.y,
         base_state.pose.theta * M_PI / 180, base_state.twist.vx,
         base_state.twist.vy, base_state.twist.w);
}

void CMotor::debugMotorPid(int i) {
  if (i < 0 || i > 3) return;

  printf("CMotor::Motor %d:: Pid:: input:%f setpoint:%f output:%f\n", i,
         motor_pids[i].input, motor_pids[i].setpoint, motor_pids[i].output);
}

void CMotor::set_yaw(float _yaw, CMotorAngualarDirectionType dir) {
  yaw_pid.setpoint = toYawPidSetpoint(_yaw, yaw_pid.input, dir);
}
