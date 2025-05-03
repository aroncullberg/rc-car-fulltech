//
// Created by aron on 5/3/2025.
//

#include "console_engine.h"


#include <cstring>
#include <cstdarg>

#include "esp_console.h"
#include "linenoise/linenoise.h"

#include "console_transport.h"

#include "console_engine.h"

using namespace console;

Engine& Engine::instance()
{
    static Engine instance;
    return instance;
}

esp_err_t Engine::init(size_t history_lines)
{
    print_mux_ = xSemaphoreCreateMutex();
    if (!print_mux_) return ESP_ERR_NO_MEM;

    esp_console_config_t cfg{};
    cfg.max_cmdline_length = 256;
    cfg.hint_color = 35;
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_console_init(&cfg));

    linenoiseHistorySetMaxLen(static_cast<int>(history_lines));
    return ESP_OK;
}

void Engine::register_transport(ITransport* transport)
{
    if (transport_count_ < k_max_transports) {
        transports_[transport_count_++] = transport;
    }
}

void Engine::execute_line(const char* line)
{
    int ret = esp_console_run(line, nullptr);
    (void) ret;
    // TODO: handle errors/send back to sender
}

void Engine::write_line(const char* line, ...)
{
    va_list args;
    va_start(args, line);
    vprintf(line, args);
    va_end(args);
}

void Engine::vprintf(const char* format, va_list args)
{
    if (xSemaphoreTake(print_mux_, pdMS_TO_TICKS(50)) == pdTRUE) {
        char buffer[256];
        int len = vsnprintf(buffer, sizeof(buffer), format, args);
        for (auto* transport: transports_) {
            transport->write(buffer, len);
        }
        xSemaphoreGive(print_mux_);
    }
}

void ITransport::on_line(const char* line) {
    Engine::instance().execute_line(line);
}


