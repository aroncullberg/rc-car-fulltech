//
// Created by aron on 2025-05-01.
//

#include "measure.hpp"

#include <algorithm>
#include <sys/types.h>

using namespace profiler;

void Measure::start() {
    const uint64_t now = esp_timer_get_time();
    if (count_ == 0) t0_ = now;
    last_start_ = now;
}

void Measure::end()
{
    const uint64_t now = esp_timer_get_time();
    const uint64_t delta_us = now - last_start_;

    total_time_us_ += delta_us;
    count_++;

    const size_t bin = delta_us / k_bin_size_us;
    bins_[std::min(bin, k_overflow_bin) - 1] = bin;
}

void Measure::reset()
{
    count_ = 0;
    total_time_us_ = 0;
    t0_ = 0;
    last_start_ = 0;
    // std::fill(std::begin(bins_), std::end(bins_), 0);
}

uint32_t Measure::count() const
{
    return count_;
}

uint64_t Measure::total_time_us() const
{
    return total_time_us_;
}

uint64_t Measure::elapsed_us() const
{
    return esp_timer_get_time() - t0_;
}

uint16_t Measure::duty_cycle_permille() const
{
    if (count_ == 0) return 0;
    const uint64_t elapsed = elapsed_us();
    return elapsed ? static_cast<uint16_t>((total_time_us_ * 1000) / elapsed) : 0;
}

uint32_t Measure::average_freq_mill_hz() const
{
    const uint64_t elapsed = elapsed_us();
    return elapsed ? static_cast<uint32_t>(count_ * 1000000000ULL / elapsed) : 0;
}

uint64_t Measure::percentile(const percentile_t p) const
{
    if (p > 100) return 0;
    if (count_ == 0) return 0;

    const uint32_t target = (count_ * p + 99) / 100;

    uint32_t cum = 0; // haha 😳
    for (size_t i = 0; i < k_num_bins; ++i) {
        cum += bins_[i];
        if (cum >= target) {
            return (i + 1) * k_bin_size_us;
        }
    }
    return k_overflow_bin * k_bin_size_us;
}











