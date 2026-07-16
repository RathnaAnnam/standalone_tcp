#pragma once

#include <cmath>

namespace angles
{

inline double normalize_angle(double angle)
{
    const double TWO_PI = 2.0 * M_PI;

    while (angle > M_PI)
    {
        angle -= TWO_PI;
    }

    while (angle < -M_PI)
    {
        angle += TWO_PI;
    }

    return angle;
}

}  // namespace angles
