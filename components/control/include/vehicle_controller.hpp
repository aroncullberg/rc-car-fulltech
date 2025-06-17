//
// Created by aron on 5/9/2025.
//

#pragma once

#include <pid_controller.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu.hpp"
#include "rclink.h"
#include "gps.h"
#include "json_config.h"
#include "DShotRMT.h"
#include "servo.h"


namespace controller
{

class VehicleController {
public:
    struct Config
    {
        Servo servo;
        DShotRMT motor_fr;
        DShotRMT motor_fl;
        DShotRMT motor_rl;
        DShotRMT motor_rr;
    };

    explicit VehicleController(const Config& config);
    ~VehicleController();

    VehicleController(const VehicleController&) = delete;
    VehicleController& operator=(const VehicleController&) = delete;

    esp_err_t init();
    esp_err_t start();
    void stop();
private:
    static void cl_main_task_entry(void* arg);
    static void cl_steering_task_entry(void* arg);
    static void cl_motor_task_entry(void* arg);

    [[noreturn]] void cl_main_task();

    void update_rpm_telemetry();

    bool armed{false};

    TaskHandle_t cl_main_task_handle{nullptr};

    int16_t steering_range_low{0};
    int16_t steering_range_high{0};
    int16_t steering_center{1500};

    bool reverse{false};
    Config config;
};
}


