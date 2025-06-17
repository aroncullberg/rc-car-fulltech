//
// Created by aron on 5/9/2025.
//

#include "vehicle_controller.hpp"

#include <esp_check.h>
#include <vehicle_data.hpp>

#define NOT !

#define poop get

static constexpr auto tag = "VehicleDynamics";
using namespace controller;

using motion::Imu;
using nav::Gps;
using rc::Receiver;

VehicleController::VehicleController(const Config& config): config(){}

VehicleController::~VehicleController()
{
    stop();
}

esp_err_t VehicleController::init()
{
    config.motor_fl.begin_UNSAFE();
    config.motor_fr.begin_UNSAFE();
    config.motor_rl.begin_UNSAFE();
    config.motor_rl.begin_UNSAFE();
    return ESP_OK;
}

esp_err_t VehicleController::start()
{
    BaseType_t ret = xTaskCreatePinnedToCore(
        cl_main_task_entry,
        "cl_main_task",
        8000,
        this,
        5,
        &cl_main_task_handle,
        1);

    ESP_RETURN_ON_FALSE(ret == pdPASS, ret, tag, "Failed to create cl_main_task");

    return ESP_OK;
}

void VehicleController::stop()
{
    if (cl_main_task_handle) {
        vTaskDelete(cl_main_task_handle);
        cl_main_task_handle = nullptr;
    }
}

void VehicleController::cl_main_task_entry(void* arg)
{
    static_cast<VehicleController*>(arg)->cl_main_task();
}


void VehicleController::cl_main_task()
{
    TickType_t last_wake = xTaskGetTickCount();

    constexpr auto ch_throttle                      = rc::ChannelIndex::CH1;
    constexpr auto ch_steering                      = rc::ChannelIndex::CH2;
    constexpr auto ch_arm                           = rc::ChannelIndex::CH3;
    constexpr auto ch_low_steering_range            = rc::ChannelIndex::CH4;
    constexpr auto ch_high_steering_range           = rc::ChannelIndex::CH5;
    constexpr auto ch_steeroffset                   = rc::ChannelIndex::CH6;
    constexpr auto ch_wide_steer                    = rc::ChannelIndex::CH7;
    constexpr auto ch_reverse                       = rc::ChannelIndex::CH8;

    const Receiver& rc = Receiver::instance();

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));

        if (!rc.valid_data()) {
            continue;
        }

        // Arming
        if (
            rc.get(ch_throttle).low() && !armed && rc.get(ch_arm).high()) {
            armed = true;
        }
        else if (!rc.get(ch_arm).high() && armed) {
            armed = false;
        }

        if (!armed) {
            steering_range_low = rc.get(ch_low_steering_range).raw() / 10;
            steering_range_high = rc.get(ch_high_steering_range).raw() / 5;
            steering_center = (rc.get(ch_steeroffset).raw() - 1000) / 10 + 1500;
            reverse = rc.get(ch_reverse).high();
        }

        if (rc.get(ch_wide_steer).high()) {
            config.servo.setPosition(rc.get(ch_steering).raw(), steering_range_high, steering_center);
        } else {
            config.servo.setPosition(rc.get(ch_steering).raw(), steering_range_low, steering_center);
        }

        if (reverse) {
            int16_t dshot_value = 1000 - rc.get(ch_throttle).raw() / 2;
            config.motor_fl.sendThrottle(dshot_value);
            config.motor_fr.sendThrottle(dshot_value);
            config.motor_rl.sendThrottle(dshot_value);
            config.motor_rr.sendThrottle(dshot_value);
        } else {
            int16_t dshot_value = 1000 + rc.get(ch_throttle).raw() / 2;
            config.motor_fl.sendThrottle(dshot_value);
            config.motor_fr.sendThrottle(dshot_value);
            config.motor_rl.sendThrottle(dshot_value);
            config.motor_rr.sendThrottle(dshot_value);
        }

        uint32_t rpm_fr = config.motor_fl.getErpm();
        uint32_t rpm_fl = config.motor_fr.getErpm();
        uint32_t rpm_rl = config.motor_rl.getErpm();
        uint32_t rpm_rr = config.motor_rr.getErpm();

        if (rpm_fr != 0 || rpm_fr != 65535) {
            rpm_fr = VehicleData::instance().get_motor_erpm()[0];
        }
        if (rpm_fl != 0 || rpm_fl != 65535) {
            rpm_fl = VehicleData::instance().get_motor_erpm()[1];
        }
        if (rpm_rl != 0 || rpm_rl != 65535) {
            rpm_rl = VehicleData::instance().get_motor_erpm()[2];
        }
        if (rpm_rr != 0 || rpm_rr != 65535) {
            rpm_rr = VehicleData::instance().get_motor_erpm()[3];
        }
        std::array<int16_t, 4> erpm = {static_cast<int16_t>(rpm_fr), static_cast<int16_t>(rpm_fl), static_cast<int16_t>(rpm_rl), static_cast<int16_t>(rpm_rr)};
        VehicleData::instance().updateErpm(erpm);
    }
}

