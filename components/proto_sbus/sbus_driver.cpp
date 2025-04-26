//
// Created by cullb on 2025-04-26.
//

#include "sbus_driver.h"

using rc::ChannelValue;
using rc::ChannelIndex;

static const char *TAG = "SbusDriver";

namespace proto
{
    SbusDriver::SbusDriver(const Config &cfg) : cfg_(cfg) {
    };

    SbusDriver::~SbusDriver() {
        stop();
        ESP_LOGI(TAG, "SbusDriver instance destroyed");
    }

    esp_err_t SbusDriver::init() const {
        return configureUART();
    }

    esp_err_t SbusDriver::start() {
        if (running_) return ESP_ERR_INVALID_STATE;
        BaseType_t res = xTaskCreatePinnedToCore(
            taskEntry,
            "sbus_driver",
            4096,
            this,
            5,
            &task_,
            1
        );

        running_ = (res == pdPASS);

        return running_ ? ESP_OK : ESP_FAIL;
    }

    esp_err_t SbusDriver::stop() {
        if (!running_) return ESP_ERR_INVALID_STATE;
        vTaskDelete(task_);
        task_ = nullptr;
        running_ = false;
        return ESP_OK;
    }

    esp_err_t SbusDriver::configureUART() const {
        const uart_config_t ucfg = {
            .baud_rate = cfg_.buad_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_EVEN,
            .stop_bits = UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_param_config(cfg_.uart_num, &ucfg));
        ESP_ERROR_CHECK_WITHOUT_ABORT(
            uart_set_pin(cfg_.uart_num, cfg_.uart_tx_pin, cfg_.uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_driver_install(cfg_.uart_num,
            512,
            0,
            20,
            uart_event_queue_handle_,
            0));
        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_rx_full_threshold(cfg_.uart_num, FRAME_SIZE));
        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_rx_timeout(cfg_.uart_num, 2));
        ESP_ERROR_CHECK_WITHOUT_ABORT(uart_set_line_inverse(cfg_.uart_num, UART_SIGNAL_RXD_INV));

        return ESP_OK;
    }

    void SbusDriver::taskEntry(void *arg) {
        auto *instance = static_cast<SbusDriver *>(arg);
        instance->run();

        // static_cast<SbusDriver *>(arg)->run();
    }

    void SbusDriver::run() {
        uart_event_t e;

        // TODO: decrease indenting
        while (true) {
            xQueueReceive(*uart_event_queue_handle_, &e, portMAX_DELAY);
            switch (e.type) {
                case UART_DATA:

                    /*  Only pull bytes when the ISR tells us there are *exactly*
                    one SBUS frame (25 bytes) waiting.  Anything else is likely
                    noise, a partial frame, or multiple concatenated frames.   */

                    if (e.size == FRAME_SIZE) {
                        uint8_t buf[FRAME_SIZE];
                        int len = uart_read_bytes(cfg_.uart_num,
                                                  buf,
                                                  FRAME_SIZE,
                                                  0); // no extra wait
                        if (len == FRAME_SIZE && buf[0] == START_BYTE &&
                            buf[FRAME_SIZE - 1] == END_BYTE) {
                            processFrame(buf, FRAME_SIZE);
                        } else {
                            uart_flush_input(cfg_.uart_num); // bad frame
                        }
                    } else {
                        /* Not the size we expect → drop & flush to keep FIFO clean */
                        uart_flush_input(cfg_.uart_num);
                    }
                    break;
                case UART_FIFO_OVF:
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "UART overflow-flushing");
                    uart_flush_input(cfg_.uart_num);
                    xQueueReset(*uart_event_queue_handle_);
                    break;

                default:
                    break;
            }
        }
    }

    void SbusDriver::processFrame(const uint8_t *frame, size_t length) {
        if (length != FRAME_SIZE) return;

        int byte_idx = 1;
        int bit_idx = 0;

        for (uint8_t ch = 0; ch < 16; ++ch) {
            uint16_t raw = 0;
            for (int bit = 0; bit < 8; ++bit) {
                if (frame[byte_idx] & (1 << bit_idx)) raw |= (1 << bit);

                bit_idx++;
                if (bit_idx == 8) {
                    bit_idx = 0;
                    ++byte_idx;
                }
            }

            ChannelValue scaled = rawToScaled(raw);
            rc::Receiver::instance().update(static_cast<ChannelIndex>(ch), scaled);
        }
    }
}
