#ifndef AK60_HPP__
#define AK60_HPP__

#include "../common/processor_type.h"
#include "../communication/MCP2515.hpp"
#include "../maths/math.hpp"

enum class MOTOR_ERRORS : uint8_t
{
  NO_FAULT = 0,
  MOTOR_OVER_TEMP,
  OVER_CURRENT,
  OVER_VOLT,
  UNDER_VOLT,
  ENC_FAULT,
  MOS_OVER_TEMP,
  MOTOR_STALL
};

class AK60
{
public:
  AK60();

  AK60(MCP2515 _mcp2515)
      : mcp2515(_mcp2515) {};

  AK60(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_port, uint16_t _cs_pin)
      : mcp2515(_hspi, _cs_port, _cs_pin) {}

  AK60(SPI_HandleTypeDef *_hspi, GPIO_TypeDef *_cs_port, uint16_t _cs_pin, float _kp, float _kd, float _def_vel = 0.9f, float _def_toq = 0.0f)
      : mcp2515(_hspi, _cs_port, _cs_pin),
        kp(_kp),
        kd(_kd),
        def_vel(_def_vel),
        def_toq(_def_toq) {}

  bool init();

  bool deinit();

  bool set_origin();

  bool set_pos(float _pos);

  bool set_pos(float _pos, float _kp);

  bool set_vel(float _vel);

  bool set_vel(float _vel, float _kp);

  bool set_toq(float _toq);

  bool set_toq(float _toq, float _kp);

  void update_params();

  bool set_pos_vel(float _pos, float _vel);

  bool set_pos_vel_toq(float _pos, float _vel, float _toq);

  bool init_can();

  bool deinit_can();

  bool send_origin_msg();

  float get_position() { return position; }

  float get_speed() { return speed; }

  float get_torque() { return torque; }

  float get_temperature() { return tempr; }

  MOTOR_ERRORS get_motor_error() { return motor_error; }

  MCP2515 mcp2515;

private:
  float position;
  uint16_t motor_id = 0x01;
  float speed;
  float torque;
  float tempr;
  MOTOR_ERRORS motor_error;
  uint8_t ctrl_252_count = 0; // counter for 252 status code
  bool is_can_running = false;

  uCAN_MSG tx_msg, rx_msg;

  float kp, kd;
  float def_vel = 0.0f;
  float def_toq = 10.0f;

  /// limit data to be within bounds ///
  float P_MIN = -12.5f;
  float P_MAX = 12.5f;
  float V_MIN = -30.0f;
  float V_MAX = 30.0f;
  float T_MIN = -15.0f;
  float T_MAX = 15.0f;
  float KP_MIN = 0;
  float KP_MAX = 500.0f;
  float KD_MIN = 0;
  float KD_MAX = 5.0f;
  float Test_Pos = 0.0f;
  float I_MAX = 40.0f;

  void pack_cmd(uCAN_MSG *msg, float p_des, float v_des, float kp, float kd, float t_ff);
  void unpack_reply(uCAN_MSG msg);
};

#endif