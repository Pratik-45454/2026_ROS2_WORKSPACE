#include "as5600_adc.hpp"

bool AS5600_ADC::init()
{
  if (hadc == nullptr)
    return false;

  if (HAL_ADC_Start_DMA(hadc, &data_buffer, 1) == HAL_OK)
    return true;

  return false;
}

void AS5600_ADC::adc_callback()
{

  rad_angle = (float)data_buffer * F32_2_PI / 4096.0f; // [0, 2π)
  deg_angle = rad_angle * RAD2DEG;

  float wrapped_angle = wrapAngle(rad_angle); // convert to [-π, π]

  if (first_reading)
  {
    prev_wrapped_angle = wrapped_angle;
    first_reading = false;
  }
  else
  {
    cumulative_angle += angleChange(wrapped_angle, prev_wrapped_angle);
    prev_wrapped_angle = wrapped_angle;
  }

  HAL_ADC_Start_DMA(hadc, &data_buffer, 1);
}
