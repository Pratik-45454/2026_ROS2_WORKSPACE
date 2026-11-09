#include <cstdint>

#include "as5600.hpp"

AS5600::AS5600(I2C_HandleTypeDef *_i2ch) : i2ch(_i2ch) {}

bool is_first_callback = true;

bool AS5600::init()
{
  bool to_return = true;
  to_return &= receive();

  return to_return;
}

void AS5600::rx_callback()
{
  float angle_diff = 0.0f;

  uint16_t angle_raw_value = (uint16_t)(angle_buffer[0] << 8 | angle_buffer[1]);
  float new_angle = ((float)angle_raw_value / 4096) * 360.00f;

  if (is_first_callback)
  {
    if (new_angle != 0)
    {
      angle = new_angle;
      cumulative_angle = new_angle;
      is_first_callback = false;
    }
    else
    {
      is_first_callback = true;
    }
  }
  else
  {
    if ((angle > 180.0f) && (new_angle < (angle - 180.0f)))
    {
      angle_diff = 360.0f - angle + new_angle;
    }
    else if ((new_angle > 180.0f) && (angle < (new_angle - 180.0f)))
    {
      angle_diff = new_angle - 360.0f - angle;
    }
    else
    {
      angle_diff = new_angle - angle;
    }

    cumulative_angle += angle_diff;
    angle = new_angle;
  }

  last_receive_cplt_tick = HAL_GetTick();
}

I2C_TypeDef *AS5600::get_i2c_instance() { return i2ch->Instance; }

bool AS5600::connected()
{
  return (HAL_GetTick() - last_receive_cplt_tick < I2C_CONNECTION_TIMEOUT);
}

float AS5600::get_cumulative_angle_degrees() { return cumulative_angle; }

float AS5600::get_number_of_revolutions() { return cumulative_angle / 360.0f; }

bool AS5600::receive()
{
  return HAL_I2C_Mem_Read_DMA(i2ch, AS5600_ADDR << 1, AS5600_ANGLE,
                              I2C_MEMADD_SIZE_8BIT, angle_buffer,
                              sizeof(angle_buffer)) == HAL_OK;
}

uint8_t AS5600::get_status()
{
  if (HAL_I2C_Mem_Read(i2ch, AS5600_ADDR << 1, AS5600_STATUS,
                       I2C_MEMADD_SIZE_8BIT, extra_buffer, 1, 1) == HAL_OK)
  {
    return extra_buffer[0];
  }
  else
  {
    return 0;
  }
}

uint8_t AS5600::get_agc()
{
  if (HAL_I2C_Mem_Read(i2ch, AS5600_ADDR << 1, AS5600_AGC, I2C_MEMADD_SIZE_8BIT,
                       extra_buffer, 1, 1) == HAL_OK)
  {
    return extra_buffer[0];
  }
  else
  {
    return 0;
  }
}

uint16_t AS5600::get_magnitude()
{
  if (HAL_I2C_Mem_Read(i2ch, AS5600_ADDR << 1, AS5600_MAGNITUDE,
                       I2C_MEMADD_SIZE_8BIT, extra_buffer, 2, 1) == HAL_OK)
  {
    return (extra_buffer[0] << 8) | extra_buffer[1];
  }
  else
  {
    return 0;
  }
}

float AS5600::get_angle_degrees() { return angle; }
