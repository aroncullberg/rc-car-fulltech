//
// Created by aron on 2025-05-01.
//

#pragma once

#include <esp_timer.h>

namespace profiler
{
using percentile_t = uint8_t;

class Measure
{
public:
    static constexpr size_t k_num_bins = 32;
    static constexpr uint64_t k_bin_size_us = 50;
    static constexpr size_t k_overflow_bin = k_num_bins - 1;

    Measure() {reset();}

    void start();
    void end();
    void reset();

    [[nodiscard]] uint32_t count() const;
    [[nodiscard]] uint64_t total_time_us() const;
    [[nodiscard]] uint64_t elapsed_us() const;
    [[nodiscard]] uint16_t  duty_cycle_permille() const;
    [[nodiscard]] uint32_t  average_freq_mill_hz() const;

    uint64_t percentile(percentile_t p) const;

private:
    uint32_t count_{0};
    uint64_t total_time_us_{0};
    uint64_t t0_{0};
    uint64_t last_start_{0};
    uint32_t bins_[k_num_bins]{};

};
}
