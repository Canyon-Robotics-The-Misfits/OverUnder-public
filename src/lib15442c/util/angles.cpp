#include "angles.hpp"
#include <math.h>

float lib15442c::angle_error(float angle_a, float angle_b)
{
    float wrapped_a = wrap_angle_180_180(angle_a);
    float wrapped_b = wrap_angle_180_180(angle_b);

    float error = wrapped_a - wrapped_b;

    if (fabs(error) > 180)
        error -= sgn(error) * 360;

    return error;
}

float lib15442c::wrap_angle_180_180(float angle)
{
    if (isnan(angle))
        return angle;

    return fmod(angle + 180, 360) - 180;
}

float lib15442c::wrap_angle_0_360(float angle)
{
    if (isnan(angle))
        return angle;

    float wrapped = fmod(angle, 360);
    if (wrapped < 0)
        wrapped += 360;
    return wrapped;
}