#ifndef __AS5600_ADC
#define __AS5600_ADC

#include <cstdint>

#include "../common/processor_type.h"
#include "../maths/math.hpp"
#include "adc.h"

class AS5600_ADC
{
private:
  ADC_HandleTypeDef *hadc;
  uint32_t channel;

  uint32_t data_buffer;
  float deg_angle;
  float rad_angle, prev_wrapped_angle;
  float cumulative_angle;
  bool first_reading;

public:
  AS5600_ADC() = default;
  AS5600_ADC(ADC_HandleTypeDef *_hadc) : hadc(_hadc) {};

  ~AS5600_ADC() = default;

  bool init();
  float get_angle_degrees() { return deg_angle; };
  float get_angle_radians() { return rad_angle; };
  float get_cumulative_angle_radians() { return cumulative_angle; }
  void adc_callback();
  ADC_TypeDef *get_adc_instance() { return hadc->Instance; }
};

#endif