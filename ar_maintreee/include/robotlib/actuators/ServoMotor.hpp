#ifndef SERVO_MOTOR_HPP
#define SERVO_MOTOR_HPP

#include "../controllers/pid.hpp"
#include "../sensors/encoder.hpp"
#include "maths/math.hpp"
#include "motor.hpp"

class ServoMotor
{
public:
  ServoMotor(const Motor _motor, const Encoder _encoder, PID _pid,
             const float _max_motor_omega, const float _max_motor_pwm,
             const float _min_pose = -20, const float _max_pose = 20,
             const uint16_t _robot_loop_period = 10.0f,
             const float _gear_ratio = 1.0f)
      : motor(_motor),
        encoder(_encoder),
        pid(_pid),
        min_pose(_min_pose),
        max_pose(_max_pose),
        robot_loop_period(_robot_loop_period),
        max_motor_omega(_max_motor_omega),
        max_motor_pwm(_max_motor_pwm),
        gear_ratio(_gear_ratio) {}

  /**
   * @brief Default Constructor
   */
  ServoMotor() = default;

  /**
   * @brief Copy Constructor
   */
  ServoMotor(const ServoMotor &) = default;

  /**
   * @brief Copy Assignment Operator
   */

  ServoMotor &operator=(const ServoMotor &) = default;

  /**
   * @brief Destructor
   */
  ~ServoMotor() = default;

  /**
   * @brief Initialize ServoMotor
   * @return true if init is successful else false
   */
  bool init();

  /**
   * @brief Update ServoMotor
   */
  bool update();

  /**
   * @brief Set ServoMotor Pose
   * @param _setpoint Setpoint in Radians
   */
  void set_pose_radians(const float _setpoint);

  void set_pose_degrees(const float _setpoint);

  void reset_pose(void);

  /**
   * @brief Reset ServoMotor Pose
   * @param _pose Pose in Radians
   */
  void reset_pose(float _pose);

  /**
   * @brief Get ServoMotor Setpoint
   * @return Setpoint in Radians
   */
  float get_setpoint() const { return setpoint; }

  /**
   * @brief Get ServoMotor Minimum Pose
   * @return Minimum Pose in Radians
   */
  float get_min_pose() const { return min_pose; }

  /**
   * @brief Get ServoMotor Maximum Pose
   * @return Maximum Pose in Radians
   */
  float get_max_pose() const { return max_pose; }

  /**
   * @brief Get ServoMotor Output
   * @return Output
   */
  float get_output() const { return output; }
  void set_speed(float speed){
    motor.set_speed(speed);
  }
  float get_encoder_pose_degrees()
  {
    return encoder.get_calibrated_revolutions() * 360.0f * gear_ratio;
  }

  float get_encoder_pose_radians()
  {
    return encoder.get_calibrated_revolutions() * F32_2_PI * gear_ratio;
  }

  Encoder &get_encoder(void) { return encoder; }

  Motor &get_motor(void) { return motor; }

  // void calibrate_offset(float _pose) {
  //   offset = get_encoder_pose() - _pose;
  // }

  Motor motor;      /**< Motor object */
  Encoder encoder;  /**< Encoder object */
  PID pid; /**< PID object */

  float setpoint = 0.0f; /**< Setpoint in Radians */
  float pose = 0.0f;     /**< Pose in Radians */
  float min_pose = 0.0f; /**< Minimum Pose in Radians */
  float max_pose = 0.0f; /**< Maximum Pose in Radians */
  float output = 0.0f;   /**< Output */
  uint16_t robot_loop_period;
  float max_motor_omega;
  float max_motor_pwm;
  float gear_ratio;
};

#endif // SERVO_MOTOR_HPP
