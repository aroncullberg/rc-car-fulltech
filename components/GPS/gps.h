#pragma once

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "TinyGPS++.h"

#include <functional>

#include "sensor_types.h"
#include "system_types.h"

#define MATCH 0

namespace sensor {

class GPS {
public:
    struct Config {
        uart_port_t uart_num{UART_NUM_1};
        gpio_num_t uart_tx_pin{GPIO_NUM_NC};
        gpio_num_t uart_rx_pin{GPIO_NUM_NC};
        int baud_rate{38400};
        size_t rx_buffer_size{2048};
        size_t tx_buffer_size{0};
        uint16_t frequency{30};
        Frequency targetFreq{Frequency::F10Hz};
    };

    GPS(const Config& config);
    ~GPS();

    esp_err_t init();
    esp_err_t start();
    esp_err_t stop();

    void updateFromConfig();

private:
    static constexpr const char* TAG = "GPS";

    void processGPSData();  // New method to handle data updates

    esp_err_t configureUART();

    static void gpsTask(void* parameters);
    static void reportingTask(void* parameters);
    TaskHandle_t reporting_task_handle_{nullptr};
    TaskHandle_t task_handle_{nullptr};

    bool reporting_running_{false};

    TinyGPSPlus tiny_gps_;
    Config config_;
    GpsData current_data{}; // TODO: rename to current_data_
    uint32_t max_speed_mmps_{0}; // Maximum speed in mm/s

    std::function<void()> callback_;

    bool is_running{false};
    bool debug_logging_{false};
    bool verbose_logging_{false};
    bool log_freq_{false};
};

}