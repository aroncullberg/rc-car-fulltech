//
// Created by aron on 2025-04-30.
//

#pragma once

#include <cstdint>

namespace motion {

struct Accel {
    int16_t x{0};
    int16_t y{0};
    int16_t z{0};
};

struct Gyro {
    int16_t x{0};
    int16_t y{0};
    int16_t z{0};
};

    static constexpr float GYRO_TO_DPS = 500.0f / 32768.0f;


struct Quat6 {
    int32_t x{0};
    int32_t y{0};
    int32_t z{0};
};

struct Quat9 {
    int32_t x{0};
    int32_t y{0};
    int32_t z{0};
    int16_t accuracy{0};
};

struct Stats {
  bool valid_data{false};
  // todo: add more stats
 };

}