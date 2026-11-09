#include "bldc.hpp"
#include "../maths/math.hpp"

BLDC::BLDC() : htim(nullptr) {}

BLDC::BLDC(TIM_HandleTypeDef *_htim,
           int _timer_pwm_channel,
           float _min_time,
           float _max_time,
           float _time_period)
    : htim(_htim),
      timer_pwm_channel(_timer_pwm_channel),
      min_time(_min_time),
      max_time(_max_time),
      time_period(_time_period)
{
}

bool BLDC::init()
{
  if (htim == nullptr)
    return false;

  if (HAL_TIM_PWM_Start(htim, timer_pwm_channel) != HAL_OK)
    return false;

  __HAL_TIM_SetCompare(htim, timer_pwm_channel, htim->Instance->ARR * 0.05f);
  init_start_time = HAL_GetTick();

  return true;
}

bool BLDC::is_ready()
{
  if ((HAL_GetTick() - init_start_time) < 2000)
    return false;

  return true;
}

void BLDC::set_speed(float _speed)
{
  if (is_ready())
  {
    speed = clamp<float>(_speed, 0.0f, 1.0f);
    duty = min_time + (max_time - min_time) * speed;
    __HAL_TIM_SetCompare(htim, timer_pwm_channel, (uint16_t)(htim->Instance->ARR * duty / time_period));
  }
}
