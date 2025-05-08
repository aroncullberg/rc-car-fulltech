//
// Created by aron on 5/7/2025.
//

#include "task_stats.hpp"

using namespace diagnostics;

TaskStats& TaskStats::instance()
{
    static TaskStats instance;
    return instance;
}

void TaskStats::update(const std::vector<RuntimeStats>&& v)
{
    auto snapshot = std::make_shared<const std::vector<RuntimeStats>>(std::move(v));

    tasks.store(snapshot, std::memory_order_release);
}

std::vector<RuntimeStats> TaskStats::get() const
{
    auto snapshot = tasks.load(std::memory_order_acquire);
    if (!snapshot) return {};
    return *snapshot;
}