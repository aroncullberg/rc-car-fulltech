//
// Created by aron on 5/3/2025.
//

#include "console_engine.h"

#include "usb_transport.h"

#include <esp_check.h>
#include <driver/usb_serial_jtag_vfs.h>

static const char* TAG = "usb_transport";

using namespace console;



esp_err_t UsbTransport::start()
{
    if (task_) return ESP_ERR_INVALID_STATE;

    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();

    cfg.tx_buffer_size = 512;
    cfg.rx_buffer_size = 512;
    ESP_RETURN_ON_ERROR(usb_serial_jtag_driver_install(&cfg), TAG, "usb_serial_jtag_driver_install fail");

    BaseType_t ok = xTaskCreatePinnedToCore(task_entry,
        TAG, 4096, this, 5, &task_, 0);
    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

void UsbTransport::task_entry(void* arg)
{
    static_cast<UsbTransport *>(arg)->rx_loop();
}

void UsbTransport::rx_loop()
{
    while (true) {
        uint8_t c;
        int len = usb_serial_jtag_read_bytes(&c, 1, portMAX_DELAY);

        usb_serial_jtag_write_bytes(&c, 1, portMAX_DELAY);

        if (len <= 0) continue;
        if (c == '\r' || c == '\n') {
            linebuf_[pos_] = '\0';
            on_line(linebuf_);
            pos_ = 0;
        } else if (pos_ < k_line_buf_len - 1) {
            linebuf_[pos_++] = static_cast<char>(c);
        }
    }
}

void UsbTransport::write(const char* data, size_t len)
{
    usb_serial_jtag_write_bytes(reinterpret_cast<const uint8_t*>(data), len, portMAX_DELAY);
}

