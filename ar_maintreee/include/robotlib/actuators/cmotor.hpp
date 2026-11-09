/***********************************************************************************************
 * @file cmotor.hpp
 * @brief Header file for the CMotor class providing comblete base control for a
 *robot.
 * @author Robotics Team, Robotics Club Pulchowk Campus
 * @date 2023
 *
 * @details
 * This file declares the CMotor class, which represents the control system
 * for a robot equipped with motors, encoders, PIDs, omniwheel kinematics,
 * freewheel and imu.
 **********************************************************************************************/

#ifndef _COMPLETE_MOTOR_HPP
#define _COMPLETE_MOTOR_HPP

#include "../actuators/motor.hpp"
#include "../communication/uart.hpp"
#include "../controllers/discrete_pid.hpp"
#include "../controllers/pid.hpp"
#include "../filters/exponential.hpp"
#include "../kinematics/omniwheel.hpp"
#include "../msgs/sensor_msg.hpp"
#include "../msgs/state_msg.hpp"
#include "../sensors/encoder.hpp"
#include "motor.hpp"

enum YawPidState { YAW_PID_IDLE, YAW_PID_WAIT, YAW_PID_INIT, YAW_PID_ACTIVE };

enum CMotorAngualarDirectionType {
  CMADT_AUTO,
  CMADT_CLOCKWISE,
  CMADT_ANTICLOCKWISE
};

/**
 * @class CMotor
 * @brief Class representing the control system for a robot with motors,
 * encoders, PIDs, and omniwheel kinematics.
 */
class CMotor {
 public:
  /**
   * @brief Constructor for the CMotor class.
   * @param _motors Array of Motor objects.
   * @param _encoders Array of Encoder objects.
   * @param _kinematics OmniwheelKinematics object.
   * @param _motor_pids Array of PID objects for motor control.
   * @param _max_motor_omega Maximum allowed motor angular velocity.
   * @param _max_motor_pwms Array of maximum PWM values for each motor.
   * @param _motor_compute_time Time interval for the motor computation.
   */
  CMotor(const Motor *_motors, const Encoder *_encoders,
         const OmniwheelKinematics &_kinematics, const PID *_motor_pids,
         const float &_max_motor_omega, const float *_max_motor_pwms,
         const uint16_t &_motor_compute_time);

  /**
   * @brief Default constructor for the CMotor class.
   */
  CMotor() = default;

  /**
   * @brief Default copy constructor for the CMotor class.
   */
  CMotor(const CMotor &) = default;

  /**
   * @brief Default assignment operator for the CMotor class.
   */
  CMotor &operator=(const CMotor &) = default;

  /**
   * @brief Default destructor for the CMotor class.
   */
  ~CMotor() = default;

  /**
   * @brief Initializes the control system.
   */
  bool init();

  /**
   * @brief Sets the setpoint twist for the robot.
   * @param _twist Twist object representing the setpoint twist.
   */
  void setTwist(const Twist &_twist);

  /**
   * @brief Sets the setpoint base omega.
   * @param _omega Base angular velocity.
   */
  void setBaseOmega(float _omega);

  /**
   * @brief Compute PIDS and Kinematics for Motor Omegas
   * @return True if the computation done, otherwise false.
   */
  bool compute();

  void computeKinematicsFromCmdTwist();

  /**
   * @brief Actuate the motors.
   */
  void actuate();

  /**
   * @brief Stops the actuators
   */
  void stop();

  /**
   * @brief Sets the PID values for the Base Twist control.
   * @param _yaw_pid Pointer to the PID object for yaw control.
   * @param _vx_pid Pointer to the PID object for x velocity control.
   * @param _vy_pid Pointer to the PID object for y velocity control.
   */
  void setBasePids(const PID *_yaw_pid, const PID *_vx_pid, const PID *_vy_pid);

  /**
   * @brief Enables or disables the Base Twist control PIDs.
   * @param is_yaw Flag to enable or disable the yaw PID.
   * @param is_v Flag to enable or disable the velocity PID.
   * @return True if the PIDs are enabled, false otherwise.
   */
  bool enableBasePids(bool is_yaw, bool is_v);

  /**
   * @brief Activates the base twist pid control.
   * @return True if the base twist PID is activated currently, false otherwise.
   */
  bool activateBasePids();

  /**
   * @brief Deactivates the base_twist pid control.
   */
  void deactivateBasePids();

  /**
   * @brief Get the robot odom.
   * @return Odomery object representing the current state of the robot base.
   */
  Odometry getState() const;

  /**
   * @brief Set the robot odom.
   * @param _state Odometry object representing the current state of the robot
   * base.
   */
  void feedState(const Odometry &_state);

  /**
   * @brief Get the Encoders.
   * @return Array of Encoder Objects.
   */
  Encoder *getEncoders();

  /**
   * @brief Get the motors.
   * @return Array of Motor objects.
   */
  Motor *getMotors();

  /**
   * @brief Get the kinematics.
   * @return OmniwheelKinematics object.
   */
  OmniwheelKinematics &getKinematics();

  /**
   * @brief Get the twist.
   * @return Twist object representing the setpoint twist.
   */
  Twist getTwist() const;

  float *getEncoderOmegas();

  /**
   * @brief Prints the setpoint twist.
   */
  void printSetTwist() const;

  /**
   * @brief Prints the output twist.
   */
  void printOutputTwist() const;

  /**
   * @brief Prints the encoder angular velocities.
   */
  void printEncoderOmegas() const;

  /**
   * @brief Prints the encoder counts.
   */
  void printEncoderCounts();

  /**
   * @brief Prints motor set point and current omega in serial prlotter way.
   */
  void printMotorGraph(int n);

  /**
   * @brief Prints motor set point and current omega in serial prlotter way for
   * all motors.
   */
  void printMotorGraphAll();

  void printSetGraphAll();

  /**
   * @brief Prints the setpoint motor angular velocities.
   */
  void printSetpointMotorOmegas() const;

  /**
   * @brief Prints the pose and twist read from freewhel odometry.
   */
  void printBaseState() const;

  /**
   * @brief Prints the PID output values.
   */
  void printMotorPidOutput() const;

  void debugMotorPid(int i);

  void set_yaw(float _yaw, CMotorAngualarDirectionType dir = CMADT_AUTO);

  PID *get_yaw_pid() { return &yaw_pid; }

  bool is_yaw_set_reached = false;

 private:
  Motor motors[4];                /**< Array of Motor objects. */
  Encoder encoders[4];            /**< Array of Encoder objects. */
  OmniwheelKinematics kinematics; /**< Omniwheel kinematics object. */

  PID motor_pids[4]; /**< Array of Pid objects for motor control. */
  PID yaw_pid;
  PID vx_pid;
  PID vy_pid;
  // ExponentialFilter<float> encoder_filters[4]{{0.5f}, {0.5f}, {0.5f},
  // {0.5f}};

  float max_motor_omega;   /**< Maximum allowed motor angular velocity. */
  float max_motor_pwms[4]; /**< Array of maximum PWM values for each motor. */
  uint16_t motor_compute_time; /**< Time interval for the motor compute. */

  float motor_omegas[4] = {0, 0, 0,
                           0}; /**< Array of motor angular velocities. */
  float encoder_omegas[4] = {0, 0, 0, 0};
  float output_motor_omegas[4] = {0, 0, 0, 0};

  Twist cmd_twist{0, 0, 0};    /**< Base twist object. */
  Twist output_twist{0, 0, 0}; /**< Twist object for motor control. */
  uint32_t motor_compute_tick; /**< Motor compute tick. */

  bool is_yaw_pid_set = false; /**< Flag to check if yaw PID is set. */
  bool is_vx_pid_set = false;  /**< Flag to check if x velocity PID is set. */
  bool is_vy_pid_set = false;  /**< Flag to check if y velocity PID is set. */
  bool is_yaw_pid_enabled = false; /**< Flag to enable yaw PID control. */
  bool is_v_pid_enabled = false; /**< Flag to enable x velocity PID control. */
  bool is_base_pids_active =
      false; /**< Flag to check if base twist PID is active. */

  Odometry base_state{{0, 0, 0}, {0, 0, 0}}; /**< Robot state object. */
  float prev_yaw = 0;                        /**< Previous yaw value. */
  YawPidState yawPidState = YAW_PID_IDLE;    /**< Yaw PID state. */
  uint16_t wait_count = 0;
};

#endif  // _CONTROL_MOTOR_HPP
