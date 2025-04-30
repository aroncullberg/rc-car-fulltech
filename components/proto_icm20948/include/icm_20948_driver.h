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
    #include "icm20948_spi.h"
}

#include "imu.h"


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
            int queue_len{4};

            icm20948_accel_config_fs_sel_e accel_fsr{GPM_4};
            icm20948_gyro_config_1_fs_sel_e gyro_fsr{DPS_500};
        };

        explicit Icm20948Driver(const Config &config);

        esp_err_t init();
        esp_err_t start();
        void stop();

        Icm20948Driver(const Icm20948Driver &) = delete;
        Icm20948Driver &operator=(const Icm20948Driver &) = delete;

    private:
        static void IRAM_ATTR isrThunk(void *arg);
        void        IRAM_ATTR isr();

        static void taskEntry(void *arg);
        void run();                     // drains queue, reads FIFIO

        esp_err_t configureSpi();
        esp_err_t configureSensor(); // WHOAMI, ID, DMP, ECT.
        esp_err_t configureInterruptPin();
        esp_err_t configureDmp();

        esp_err_t setFullScaleRanges();

        Config cfg_;
        spi_device_handle_t spi_{nullptr};
        icm20948_device_t device_{};

        TaskHandle_t task_{nullptr};
        QueueHandle_t int_queue_{nullptr};

        static constexpr const char *TAG = "Icm20948Driver";

    };
}
