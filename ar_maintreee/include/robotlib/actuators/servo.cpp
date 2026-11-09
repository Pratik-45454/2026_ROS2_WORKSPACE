#include "servo.hpp"
#include "../maths/math.hpp"

Servo::Servo() : htim(nullptr) {}

Servo::Servo(TIM_HandleTypeDef *_htim,
             int _timer_pwm_channel,
             float _initial_pose,
             float _min_pose,
             float _max_pose,
             float _min_time,
             float _max_time,
             float _time_period)
    : htim(_htim),
      timer_pwm_channel(_timer_pwm_channel),
      pose(_initial_pose),
      min_pose(_min_pose),
      max_pose(_max_pose),
      min_time(_min_time),
      max_time(_max_time),
      time_period(_time_period)
{
}

bool Servo::init()
{
    if (htim == nullptr)
        return false;

    if (HAL_TIM_PWM_Start(htim, timer_pwm_channel) != HAL_OK)
        return false;

    set_pose(pose);

    return true;
}

void Servo::set_pose(float _pose)
{
    if (_pose < min_pose)
        pose = min_pose;
    else if (_pose > max_pose)
        pose = max_pose;
    else
        pose = _pose;

    duty = map<float>(pose, min_pose, max_pose, min_time, max_time);
    uint16_t count = htim->Instance->ARR * duty / time_period;
    __HAL_TIM_SetCompare(htim, timer_pwm_channel, count);
}