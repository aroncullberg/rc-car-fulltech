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
    int16_t x{0};
    int16_t y{0};
    int16_t z{0};
    uint16_t frequency{0};
    uint32_t dt{0};
};

/**
 * use  deg/s
 */
struct Gyro
{
    int16_t x{0};
    int16_t y{0};
    int16_t z{0};
    uint16_t frequency{0};
    uint32_t dt{0};
};

struct Temp
{
    uint16_t celsius{0};
};
}
