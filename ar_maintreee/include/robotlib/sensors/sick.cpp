#include "sick.hpp"
#include "adc.h"
#include "main.h"
#include <cstdint>

Sick::Sick(int sensor_num) {
  this->sensor_num = sensor_num;
  for (int i = 0; i < sensor_num; i++) {
    sensor_params[i] = {0};
  }
}

void Sick::initSensorParams(int index, uint32_t channel) {
  if (index >= 0 && index < sensor_num) {
    sensor_params[index] = {channel};
  }

  HAL_ADC_Stop(&hadc1);
}

void Sick::pollADCData() {
  ADC_ChannelConfTypeDef sConfig = {0};

  for (int i = 0; i < sensor_num; i++) {
    HAL_ADC_Stop(&hadc1);

    // Configure the channel
    sConfig.Channel = sensor_params[i].channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADC_Start(&hadc1);

    if (HAL_ADC_PollForConversion(&hadc1, 1) == HAL_OK) {
      adc_buffer[i] = HAL_ADC_GetValue(&hadc1);
    }
  }

  HAL_ADC_Stop(&hadc1);
}

uint32_t *Sick::get_raw_value() {
  pollADCData();
	return adc_buffer;
}
