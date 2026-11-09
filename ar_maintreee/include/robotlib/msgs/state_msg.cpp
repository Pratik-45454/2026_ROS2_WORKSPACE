#include "state_msg.hpp"

Quaternion Euler::toQuaternion()
{
    Quaternion q;

    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}


Euler Quaternion::toEuler()
{
    Euler ypr;
    // roll (x-axis rotation)
    float sinr_cosp = 2 * (w * x + y * z);
    float cosr_cosp = 1 - 2 * (x * x + y * y);
    ypr.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = sqrtf(1 + 2 * (w * y - x * z));
    float cosp = sqrtf(1 - 2 * (w * y - x * z));
    ypr.pitch = 2 * atan2f(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    float siny_cosp = 2 * (w * z + x * y);
    float cosy_cosp = 1 - 2 * (y * y + z * z);
    ypr.yaw = atan2f(siny_cosp, cosy_cosp);

    return ypr;
}