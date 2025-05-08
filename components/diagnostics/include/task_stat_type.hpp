//
// Created by aron on 5/7/2025.
//

#pragma once

namespace diagnostics
{
struct RuntimeStats
{
    const char* task_name{};
    uint32_t runtime_share{};
    float cpu_percent{};
    const char* status{};
    uint32_t high_water_mark{};
    uint32_t core{};
};
}