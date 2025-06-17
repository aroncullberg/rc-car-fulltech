//
// Created by aron on 5/9/2025.
//

#pragma once

#include <cstdint>
#include <tuple>

#include "vehicle_data_types.hpp"

namespace controller {


class PidController {
public:
    struct Config
    {
        float p_gain{0.0f};
        float i_gain{0.0f};
        float d_gain{0.0f};
        float output_min{-1.0f};
        float output_max{1.0f};
        float integral_limit{1.0f};
    };

    explicit PidController(const Config& c);

    // Compute PID output with automatic time calculation
    float compute(float error);

    // Compute PID with manual time delta
    float compute(float error, float dt_seconds);

    // Reset internal state
    void reset();

    // Getters and setters
    void set_gains(float p, float i, float d);
    void set_output_limits(float min, float max);
    void set_integral_limit(float limit);

    std::tuple<float, float, float> get_gains() const;

    PidData get_pid_data() const;

private:
    Config config;
    PidData pid_data{};
    float previous_error{0.0f};
    float integral{0.0f};
    uint64_t previous_time_us{0};
};


}

