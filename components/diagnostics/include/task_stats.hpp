//
// Created by aron on 5/7/2025.
//

#pragma once

#include <vector>
#include <atomic>
#include <memory>

#include "freertos/FreeRTOS.h"

#include "task_stat_type.hpp"

namespace proto
{
class TaskStatDriver;
}

namespace diagnostics
{
class TaskStats
{
public:
    static TaskStats& instance();

    std::vector<RuntimeStats> get() const;

    TaskStats(const TaskStats&) = delete;
    TaskStats& operator=(const TaskStats&) = delete;
private:
    TaskStats() = default;

    friend class proto::TaskStatDriver;

    void update(const std::vector<RuntimeStats>&& v);

    std::atomic<std::shared_ptr<const std::vector<RuntimeStats>>> tasks{nullptr};
};
}