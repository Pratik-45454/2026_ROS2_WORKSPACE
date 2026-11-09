#ifndef SICK_HPP
#define SICK_HPP

#include <cstdint>

#define MAX_SENSORS 10

class Sick {
private:
    struct SensorParams {
        uint32_t channel;
    };
    
    SensorParams sensor_params[MAX_SENSORS];
    uint32_t adc_buffer[MAX_SENSORS];
    int sensor_num;
    
    void pollADCData();
    
public:
    Sick() = default;
    Sick(int sensor_num);
    void initSensorParams(int index, uint32_t channel);
    uint32_t* get_raw_value();
};

#endif 
