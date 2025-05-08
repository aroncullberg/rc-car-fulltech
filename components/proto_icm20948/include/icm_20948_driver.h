//
// Created by aron on 2025-04-30.
//

#pragma once

#include <driver/spi_master.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_err.h>
#include <esp_log.h>


extern "C" {
    #include "icm20948.h"
    #include "icm20948_registers.h"
    #include "icm20948_enumerations.h"
    #include "icm20948_spi.h"
}

#include "imu.hpp"


namespace proto
{
    class Icm20948Driver {
    public:
        struct Config {
            spi_host_device_t spi_host{SPI3_HOST};
            gpio_num_t spi_miso{GPIO_NUM_NC};
            gpio_num_t spi_mosi{GPIO_NUM_NC};
            gpio_num_t spi_sck{GPIO_NUM_NC};
            gpio_num_t spi_cs{GPIO_NUM_NC};
            int spi_clock_hz{4 * 1000 * 1000}; // 4 MHz

            gpio_num_t int_gpio{GPIO_NUM_NC};

            uint32_t task_stack{4096};
            uint8_t task_prio{5};
            int queue_len{16};

            icm20948_accel_config_fs_sel_e accel_fsr{GPM_4}; // GPM_4 means 4g mapped to 32768 values (int16_t range)
            icm20948_gyro_config_1_fs_sel_e gyro_fsr{DPS_500}; // DPS_500 means 500 degrees per second mapped to 32768 values (uint16_t range)
        };

        explicit Icm20948Driver(const Config &config);

        esp_err_t init();
        esp_err_t start();
        void stop();

        Icm20948Driver(const Icm20948Driver &) = delete;
        Icm20948Driver &operator=(const Icm20948Driver &) = delete;

    private:
        static void IRAM_ATTR isr_thunk(void *arg);
        void        IRAM_ATTR isr();

        static void task_entry(void *arg);
        [[noreturn]] IRAM_ATTR void run();

        esp_err_t burst_read_agmt(icm20948_agmt_t& out);

        esp_err_t configure_spi();
        esp_err_t configure_sensor();
        esp_err_t configure_interrupt_pin();
        esp_err_t configure_dmp();

        esp_err_t set_full_scale_ranges();

        uint64_t last_wake_isr_{0};
        uint64_t dt{};

        Config cfg_;
        spi_device_handle_t spi_{nullptr};
        icm20948_device_t device_{};

        TaskHandle_t task_{nullptr};

        constexpr float get_accel_scale() const { return static_cast<float>(2 << cfg_.accel_fsr) / 32768.0f; }
        constexpr float get_gyro_scale() const { return static_cast<float>(25 << cfg_.gyro_fsr) / 32768.0f; }
    };
}
