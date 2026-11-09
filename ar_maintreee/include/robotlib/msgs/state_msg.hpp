#ifndef _STATE_MSG_H
#define _STATE_MSG_H

#include <math.h>
#include "arm_math.h"
#include "../common/processor_type.h"
#include "../maths/math.hpp"

#pragma pack(push, 1)

struct Twist
{
    float vx = 0.0f;
    float vy = 0.0f;
    float w = 0.0f;

    Twist(float _vx, float _vy, float _w)
        : vx(_vx), vy(_vy), w(_w) {}

    Twist() = default;

    static Twist from_v_theta_omega(const float v, const float theta, const float omega)
    {
        return Twist(v * arm_cos_f32(theta), v * arm_sin_f32(theta), omega);
    }

    float get_theta() const
    {
        return atan2(vy, vx);
    }

    void set_from_v_theta_omega(const float v, const float theta, const float omega)
    {
        vx = v * arm_cos_f32(theta);
        vy = v * arm_sin_f32(theta);
        w = omega;
    }
};

struct Position
{
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;

    Position() = default;

    Position(float _x, float _y, float _theta)
        : x(_x), y(_y), theta(_theta) {}
};

struct Odometry
{
    Position pose;
    Twist twist;
};

class Euler;
class Quaternion;

struct Euler
{
    float yaw = 0.0f;
    float pitch = 0.0f;
    float roll = 0.0f;

    Euler() = default;

    Euler(float _yaw, float _pitch, float _roll)
        : yaw(_yaw), pitch(_pitch), roll(_roll) {}

    void wrap()
    {
        yaw = wrapAngle(yaw);
        pitch = wrapAngle(pitch);
        roll = wrapAngle(roll);
    }

    Quaternion toQuaternion();
};

struct Quaternion
{
    float w = 1.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Quaternion() = default;

    void normalize()
    {
        float norm = sqrtf(w * w + x * x + y * y + z * z);
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }

    Euler toEuler();
};

#pragma pack(pop)

#endif // _STATE_MSG_H