#ifndef _BLDC_H
#define _BLDC_H

#include "../common/processor_type.h"
#include "tim.h"

class BLDC
{
public:
    BLDC();

    BLDC(TIM_HandleTypeDef *_htim,
         int _timer_pwm_channel,
         float _min_time = 1.0f,
         float _max_time = 2.0f,
         float _time_period = 20.0f);

    BLDC(const BLDC &) = default;
    BLDC &operator=(const BLDC &) = default;

    bool init();
    bool is_ready();
    bool program();
    void set_speed(float _speed);

    float get_speed() const { return speed; }
    float get_duty() const { return duty; }

    ~BLDC() = default;

private:
    TIM_HandleTypeDef *htim;
    int timer_pwm_channel;
    float min_time;
    float max_time;
    float time_period;

    uint32_t init_start_time;
    float speed;
    float duty;
};

typedef BLDC BLDC;

#endif
