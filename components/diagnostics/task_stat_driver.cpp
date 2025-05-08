//
// Created by aron on 5/7/2025.
//

#include "task_stat_driver.hpp"

#include "esp_err.h"
#include "esp_check.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

using namespace proto;
using diagnostics::RuntimeStats;

TaskStatDriver::TaskStatDriver(const Config& cfg) : config(cfg)
{
    // TODO lmao
}

esp_err_t TaskStatDriver::init()
{
    // TODO lmao
    return ESP_OK;
}

esp_err_t TaskStatDriver::start()
{
    BaseType_t ret = xTaskCreatePinnedToCore(task_entry, "task_stat_driver", config.task_stack, this, config.task_prio, &task_, 0);
    if (ret != pdPASS) {
        ESP_LOGE("task_stat", "Failed to create task");
        return ESP_FAIL;
    }
    return ESP_OK;
}

void TaskStatDriver::task_entry(void* arg)
{
    static_cast<TaskStatDriver*>(arg)->run();
}

void TaskStatDriver::run()
{
    while (true) {
        vTaskDelay(config.period);
        UBaseType_t total = uxTaskGetNumberOfTasks();
        static std::vector<TaskStatus_t> snap1(total), snap2(total);
        uint64_t rt_start = 0, rt_end = 0;

        const UBaseType_t n1 = uxTaskGetSystemState(snap1.data(), 32, &rt_start);
        vTaskDelay(config.period);
        const UBaseType_t n2 = uxTaskGetSystemState(snap2.data(), 32, &rt_end);

        const uint64_t elapsed = rt_end - rt_start;

        std::vector<RuntimeStats> out(std::max(n1, n2));

        BaseType_t i, j;
        for (i = 0; i < n1; i++) {
            for (j = 0; j < n2; j++) {
                if (snap1[i].xHandle == snap2[j].xHandle) {
                    break;
                }
            }
            if (j == n2) {
                continue; // Task not found in second snapshot
            }
            const auto& s1 = snap1[i];
            const auto& s2 = snap2[j];

            if (s1.xCoreID > 1) {
                continue;
            }

            const uint64_t dt = s2.ulRunTimeCounter - s1.ulRunTimeCounter;
            const float pct = (elapsed > 0) ? 100.0f * dt / elapsed : 0.0f;
            out[i].task_name = s1.pcTaskName;
            out[i].runtime_share = dt;
            out[i].cpu_percent = pct;


            switch (s1.eCurrentState) {
                case eRunning:
                    out[i].status = "running";
                    break;
                case eReady:
                    out[i].status = "ready";
                    break;
                case eBlocked:
                    out[i].status = "blocked";
                    break;
                case eSuspended:
                    out[i].status = "suspended";
                    break;
                case eDeleted:
                    out[i].status = "deleted";
                    break;
                case eInvalid:
                    out[i].status = "invalid";
                    break;
                default:
                    out[i].status = "unknown";
                    break;
            }
            out[i].high_water_mark = s1.usStackHighWaterMark;
            out[i].core = s1.xCoreID;
        }

        diagnostics::TaskStats::instance().update(std::move(out));
    }
}
