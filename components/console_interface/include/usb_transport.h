//
// Created by aron on 5/3/2025.
//

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/usb_serial_jtag.h"

#include "console_transport.h"

namespace console
{
class UsbTransport : public ITransport
{
public:
    esp_err_t start() override;
    void write(const char* data, size_t len) override;
private:
    static void task_entry(void* arg);
    void rx_loop();

    TaskHandle_t task_{nullptr};
    static constexpr  size_t k_line_buf_len = 256;
    char linebuf_[k_line_buf_len]{};
    size_t pos_ = 0;

};
}