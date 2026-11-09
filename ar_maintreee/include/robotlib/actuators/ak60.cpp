#include "ak60.hpp"
#include <math.h>

bool AK60::init()
{
  if (!mcp2515.init())
    return false;

  // if (!deinit_can())
  //   return false;

  // if (!send_origin_msg())
  //   return false;

  // HAL_Delay(20);

  // if (!init_can())
  //   return false;

  deinit_can();
  send_origin_msg();
  HAL_Delay(20);
  init_can();

  HAL_Delay(5);
  update_params();
  update_params();

  set_pos(position, kp);
  return true;
}

bool AK60::init_can()
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  tx_msg.frame.data0 = 0xFF;
  tx_msg.frame.data1 = 0xFF;
  tx_msg.frame.data2 = 0xFF;
  tx_msg.frame.data3 = 0xFF;
  tx_msg.frame.data4 = 0xFF;
  tx_msg.frame.data5 = 0xFF;
  tx_msg.frame.data6 = 0xFF;
  tx_msg.frame.data7 = 0xFC;

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;

  is_can_running = true;
  return true;
}

bool AK60::deinit_can()
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  tx_msg.frame.data0 = 0xFF;
  tx_msg.frame.data1 = 0xFF;
  tx_msg.frame.data2 = 0xFF;
  tx_msg.frame.data3 = 0xFF;
  tx_msg.frame.data4 = 0xFF;
  tx_msg.frame.data5 = 0xFF;
  tx_msg.frame.data6 = 0xFF;
  tx_msg.frame.data7 = 0xFD;

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;

  is_can_running = false;
  return true;
}

bool AK60::send_origin_msg()
{
  if (is_can_running == true)
    return false;

  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  tx_msg.frame.data0 = 0xFF;
  tx_msg.frame.data1 = 0xFF;
  tx_msg.frame.data2 = 0xFF;
  tx_msg.frame.data3 = 0xFF;
  tx_msg.frame.data4 = 0xFF;
  tx_msg.frame.data5 = 0xFF;
  tx_msg.frame.data6 = 0xFF;
  tx_msg.frame.data7 = 0xFE;

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;

  return true;
}

bool AK60::set_origin()
{

  if (!deinit_can())
    return false;

  if (!send_origin_msg())
    return false;

  HAL_Delay(20);

  if (!init_can())
    return false;

  HAL_Delay(5);

  update_params();
  update_params();

  // must send this or the previous command persists in the motor, and shows unexpected behaviour
  set_pos(position, kp);

  return true;
}

bool AK60::set_pos(float _pos)
{

  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, _pos, def_vel, kp, kd, def_toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
};

bool AK60::set_pos(float _pos, float _kp)
{

  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, _pos, def_vel, _kp, kd, def_toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
};

bool AK60::set_vel(float _vel)
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, 0, _vel, 0, kd, def_toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
};

bool AK60::set_vel(float _vel, float _kp)
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, 0, _vel, _kp, kd, def_toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
};

bool AK60::set_pos_vel(float _pos, float _vel)
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, _pos, _vel, kp, kd, def_toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
}

bool AK60::set_pos_vel_toq(float _pos, float _vel, float _toq)
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, _pos, _vel, kp, kd, _toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
}

bool AK60::set_toq(float _toq)
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, 0.0f, 0.0f, 0.0f, kd, _toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
}

bool AK60::set_toq(float _toq, float _kp)
{
  tx_msg.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  tx_msg.frame.id = 0x01;
  tx_msg.frame.dlc = 8;
  pack_cmd(&tx_msg, 0.0f, 0.0f, _kp, kd, _toq);

  if (mcp2515.transmit(&tx_msg) != 1)
    return false;
  return true;
}

void AK60::update_params()
{
  // init_can(); // reinitialize the CAN bus to ensure the controller is in normal mode
  mcp2515.update_ctrl_status();
  if (mcp2515.ctrlStatus.ctrl_status == 252U)
  {
    // if the controller status is 252, reinitialize the CAN bus
    // this status occurs when the tx buffers are full but the controller isnt sending anything i.e. the rx buffer is empty
    // when this occurs for more than 5 times, we reinitialize the CAN bus
    // it is not a good solution, but it works for now

    ctrl_252_count++;
    if (ctrl_252_count > 3)
    {
      deinit_can();
      init_can();
      ctrl_252_count = 0;
    }
  }
  else
  {
    ctrl_252_count = 0; // reset the counter if the controller status is not 252
  }
  mcp2515.receive(&rx_msg);
  unpack_reply(rx_msg);
}

void AK60::pack_cmd(uCAN_MSG *msg, float p_des, float v_des, float kp, float kd, float t_ff)
{
  p_des = fminf(fmaxf(P_MIN - 0.1f, p_des), P_MAX + 0.1f);
  v_des = fminf(fmaxf(V_MIN, v_des), V_MAX);
  kp = fminf(fmaxf(KP_MIN, kp), KP_MAX);
  kd = fminf(fmaxf(KD_MIN, kd), KD_MAX);
  t_ff = fminf(fmaxf(T_MIN, t_ff), T_MAX);

  /// convert floats to unsigned ints ///
  uint16_t p_int = float_to_uint(p_des, P_MIN + 0.1f, P_MAX - 0.1f, 16);
  uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  /// pack ints into the can buffer ///
  msg->frame.data0 = p_int >> 8;                           // Position High 8
  msg->frame.data1 = p_int & 0xFF;                         // Position Low 8
  msg->frame.data2 = v_int >> 4;                           // Speed High 8 bits
  msg->frame.data3 = ((v_int & 0xF) << 4) | (kp_int >> 8); // Speed Low 4 bits KP High 4 bits
  msg->frame.data4 = kp_int & 0xFF;                        // KP Low 8 bits
  msg->frame.data5 = kd_int >> 4;                          // Kd High 8 bits
  msg->frame.data6 = ((kd_int & 0xF) << 4) | (t_int >> 8); // KP Low 4 bits Torque High 4 bits
  msg->frame.data7 = t_int & 0xff;                         // Torque Low 8 bits

  if (msg->frame.data0 > 254)
    msg->frame.data0 = 254;
}

void AK60::unpack_reply(uCAN_MSG msg)
{
  /// unpack ints from can buffer ///
  uint16_t id = msg.frame.data0;                                     // Driver ID
  uint16_t p_int = (msg.frame.data1 << 8) | msg.frame.data2;         // Motor Position Data
  uint16_t v_int = (msg.frame.data3 << 4) | (msg.frame.data4 >> 4);  // Motor Speed Data
  uint16_t i_int = ((msg.frame.data4 & 0xF) << 8) | msg.frame.data5; // Motor Torque Data
  uint16_t T_int = msg.frame.data6;
  uint16_t err = msg.frame.data7;
  /// convert ints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float i = uint_to_float(i_int, -I_MAX, I_MAX, 12);
  float T = (float)T_int;

  position = p; // Read corresponding data based on ID
  speed = v;
  torque = i;
  tempr = T - 40;                               // Temperature range: -40~215
  motor_error = static_cast<MOTOR_ERRORS>(err); // Read motor error status
}