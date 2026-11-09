#ifndef AS5600_HPP
#define AS5600_HPP

#include <cstdint>

#include "arm_math.h"
#include "as5600.hpp"
#include "i2c.h"
#include "math.h"
#include "maths/math.hpp"

#define AS5600_ADDR 0x36

#define AS5600_RAW_ANGLE 0x0C
#define AS5600_ANGLE 0x0E
#define AS5600_STATUS 0x0B
#define AS5600_AGC 0x1A
#define AS5600_MAGNITUDE 0x1B

#define I2C_CONNECTION_TIMEOUT 100

class AS5600 {
 private:
  I2C_HandleTypeDef *i2ch;

  uint8_t angle_buffer[2];

  uint32_t last_receive_cplt_tick;

  float angle = 0;
  float _omega;
  float cumulative_angle = 0;

  uint8_t extra_buffer[2];

 public:
  AS5600() = default;

  AS5600(I2C_HandleTypeDef *i2ch);

  ~AS5600() = default;

  bool init();

  float get_angle_degrees();

  void rx_callback();

  I2C_TypeDef *get_i2c_instance();

  bool connected();

  float get_cumulative_angle_degrees();

  bool receive();

  float get_number_of_revolutions();

  uint8_t get_status();

  uint8_t get_agc();

  uint16_t get_magnitude();
};

#endif
