//
// Created by aron on 5/3/2025.
//

#pragma once

#include <array>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/usb_serial_jtag.h"

namespace console
{
class ITransport;

class Engine
{
public:
    static Engine& instance();

    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;

    esp_err_t init(size_t history_lines = 32);
    void register_transport(ITransport* transport);
    void execute_line(const char* line);
    void write_line(const char *line, ...); // NOTE: I really hate this but i cant see any other way of doing it

private:
    Engine() = default;

    void vprintf(const char* format, va_list args);

    static constexpr size_t k_max_transports = 4;
    std::array<ITransport*, k_max_transports> transports_{};
    size_t transport_count_ = 0;

    SemaphoreHandle_t print_mux_{nullptr};
};
}
