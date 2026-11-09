#ifndef _SERVO_H
#define _SERVO_H

#include "../common/processor_type.h"
#include "tim.h"

class Servo
{
public:
    Servo();

    Servo(TIM_HandleTypeDef *_htim,
          int _timer_pwm_channel,
          float _initial_pose = 0.0f,
          float _min_pose = 0.0f,
          float _max_pose = 180.0f,
          float _min_time = 0.5f,
          float _max_time = 2.5f,
          float _time_period = 20.0f);

    Servo(const Servo &) = default;
    Servo &operator=(const Servo &) = default;

    bool init();
    void set_pose(float _pose);

    float get_pose() const { return pose; }
    float get_duty() const { return duty; }

    ~Servo() = default;

private:
    TIM_HandleTypeDef *htim;
    int timer_pwm_channel;
    float pose;
    float min_pose;
    float max_pose;
    float min_time;
    float max_time;
    float time_period;
    float duty;
};

#endif