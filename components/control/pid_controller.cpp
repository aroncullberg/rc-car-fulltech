//
// Created by aron on 5/9/2025.
//

#include "pid_controller.hpp"

#include <esp_log.h>
#include <esp_timer.h>
#include <vehicle_data.hpp>

using namespace controller;

PidController::PidController(const Config& c) : config(c)
{
}

float PidController::compute(float error)
{
    uint64_t current_time_us = esp_timer_get_time();
    float dt = (current_time_us - previous_time_us) / 1e6f; // μs to s
    previous_time_us = current_time_us;

    if (dt < 0.2) dt = 0.2f;     // Many time bad

    float p_term = config.p_gain * error;

    integral += error * dt;
    if (integral > config.integral_limit) integral = config.integral_limit;
    if (integral < -config.integral_limit) integral = -config.integral_limit;
    const float i_term = integral;

    const float error_rate = (error - previous_error) / dt;
    previous_error = error;
    const float d_term = error_rate * config.d_gain;

    pid_data.p_term = p_term;
    pid_data.i_term = i_term;
    pid_data.d_term = d_term;
    pid_data.error = error;
    pid_data.integral = integral;
    pid_data.output = p_term + i_term + d_term;

    if (pid_data.output > config.output_max) pid_data.output = config.output_max;
    if (pid_data.output < config.output_min) pid_data.output = config.output_min;

    return pid_data.output;
}

void PidController::reset()
{
    integral = 0.0f;
    previous_error = 0.0f;
    previous_time_us = esp_timer_get_time();
}

void PidController::set_gains(float p, float i, float d)
{
    if (p < 0 || i < 0 || d < 0) {
        ESP_LOGE("pid", "gains < 0");
        return;
    }
    config.p_gain = p;
    config.i_gain = i;
    config.d_gain = d;
}

void PidController::set_output_limits(float min, float max)
{
    if (min > max) {
        ESP_LOGE("pid", "min > max");
        return;
    }
    config.output_min = min;
    config.output_max = max;
}

void PidController::set_integral_limit(float limit)
{
    if (limit < 0) {
        ESP_LOGE("pid", "integral limit < 0");
        return;
    }
    config.integral_limit = limit;
}

std::tuple<float, float, float> PidController::get_gains() const
{
    return { config.p_gain, config.i_gain, config.d_gain };
}

PidData PidController::get_pid_data() const
{
    return pid_data;
}
