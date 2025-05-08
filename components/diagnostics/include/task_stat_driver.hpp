//
// Created by aron on 5/7/2025.
//

#pragma once

#include "task_stats.hpp"

namespace proto
{
class TaskStatDriver
{
public:
    struct Config
    {
        uint32_t task_stack{4096};
        uint8_t task_prio{5};
        TickType_t period{pdMS_TO_TICKS(1000)};
    };

    explicit TaskStatDriver(const Config& cfg);

    esp_err_t init();
    esp_err_t start();

    TaskStatDriver(const TaskStatDriver&) = delete;
    TaskStatDriver& operator=(const TaskStatDriver&) = delete;

private:
    static void task_entry(void* arg);
    [[noreturn]] void run();

    TaskHandle_t task_{nullptr};
    Config config;
};
}
