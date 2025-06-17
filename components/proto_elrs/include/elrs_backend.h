//
// Created by cullb on 2025-04-26.
//

#pragma once

#include <cstdint>
#include <array>
#include <algorithm>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "rclink.h"
#include "rc_backend.h"

namespace rc {
class Backend;
}

namespace proto
{
    class ExpressLRS : public rc::Backend
    {
    public:
        struct Config {
            uart_port_t uart_num{UART_NUM_1};
            gpio_num_t uart_tx_pin{GPIO_NUM_NC};
            gpio_num_t uart_rx_pin{GPIO_NUM_NC};
        };

        explicit ExpressLRS(const Config& cfg);
        ~ExpressLRS();

        esp_err_t init();
        esp_err_t start();
        esp_err_t stop();

        ExpressLRS(const ExpressLRS&) = delete;
        ExpressLRS& operator=(const ExpressLRS&) = delete;

    private:
        static void taskEntry(void *arg);
        void run();

        static constexpr uint16_t RAW_MIN = 174;
        static constexpr uint16_t RAW_MAX = 1811;
        static constexpr uint16_t RAW_RANGE = RAW_MAX - RAW_MIN;

        inline static constexpr auto SCALE_LUT = []{
            std::array<uint16_t, RAW_RANGE + 1> lut{};
            for (uint32_t i = 0; i <= RAW_RANGE; ++i) {
                lut[i] = static_cast<uint16_t>((i * 2000u + RAW_RANGE / 2) / RAW_RANGE);
            }
            return lut;
        }();

        static constexpr rc::channel_value_t rawToScaled(const uint16_t raw) {
            const uint16_t clamped = std::clamp(raw, RAW_MIN, RAW_MAX);
            return SCALE_LUT[clamped - RAW_MIN];
        }

        /* === members === */
        Config cfg_;
        TaskHandle_t task_{nullptr};
        bool running_{false};
    };
}



