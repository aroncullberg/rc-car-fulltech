//
// Created by aron on 2025-04-30.
//

#pragma once

#include <cstdint>

namespace motion
{
/**
 * m/s²
 */
struct Accel
{
    float x{0};
    float y{0};
    float z{0};
    float frequency{0};
};

/**
 * use  deg/s
 */
struct Gyro
{
    float x{0};
    float y{0};
    float z{0};
    float frequency{0};
};
}
