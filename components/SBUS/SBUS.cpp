#include "SBUS.h"

#include <config_manager.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "data_pool.h"


namespace sensor
{
    SBUS::SBUS(const Config &config) {
        config_ = config;

        config_callback_ = [this] { this->updateFromConfig(); };
        ConfigManager::instance().registerCallback(config_callback_);
        updateFromConfig();

        ESP_LOGI(TAG, "SBUS instance created");
    }

    SBUS::~SBUS() {
        stop();
        ESP_LOGI(TAG, "SBUS instance destroyed");
    }

    esp_err_t SBUS::init() const {
        ESP_LOGI(TAG, "initializing SBUS on UART%d (TX:%d, RX:%d)",
                 config_.uart_num, config_.uart_tx_pin, config_.uart_rx_pin);

        ESP_ERROR_CHECK(configureUART());

        return ESP_OK;
    }

    void SBUS::updateFromConfig() {
        bool new_logging = ConfigManager::instance().getBool("sbus/logging", logging_);
        if (new_logging != logging_) {
            ESP_LOGI(TAG, "IMU log gyro changed: %s -> %s",
                     logging_ ? "true" : "false",
                     new_logging ? "true" : "false");
            logging_ = new_logging;
        }

        bool new_log_raw_ = ConfigManager::instance().getBool("sbus/log_raw", log_raw_);
        if (new_log_raw_ != log_raw_) {
            ESP_LOGI(TAG, "SBUS log raw changed: %s -> %s",
                     log_raw_ ? "true" : "false",
                     new_log_raw_ ? "true" : "false");
        }
    }

    esp_err_t SBUS::configureUART() const {
        const uart_config_t uart_config = {
            .baud_rate = config_.baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_EVEN,
            .stop_bits = UART_STOP_BITS_2,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_APB,
        };

        // Configure UART parameters
        esp_err_t err = uart_param_config(config_.uart_num, &uart_config);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART parameter configuration failed");
            return err;
        }

        // Set pins
        err = uart_set_pin(config_.uart_num,
                           config_.uart_tx_pin,
                           config_.uart_rx_pin,
                           UART_PIN_NO_CHANGE, //No RTS
                           UART_PIN_NO_CHANGE); //No CTS
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART pin configuration failed");
            return err;
        }

        constexpr int rx_buffer_size = 512;
        constexpr int tx_buffer_size = 0;

        err = uart_driver_install(config_.uart_num, rx_buffer_size, tx_buffer_size, 0, nullptr, 0);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "UART driver installation failed");
            return err;
        }

        // enable signal inversion (sbus inverted yadayada)
        err = uart_set_line_inverse(config_.uart_num, UART_SIGNAL_RXD_INV);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set UART signal inversion");
            return err;
        }

        return ESP_OK;
    }

    esp_err_t SBUS::start() {
        if (is_running_) {
            ESP_LOGW(TAG, "SBUS already running");
            return ESP_ERR_INVALID_STATE;
        }

        BaseType_t task_created = xTaskCreatePinnedToCore(
            sbusTask, // task function
            "sbus_task", // task name
            4096, // stack
            this, // ???
            4, // Task priority
            &task_handle_, // self-explanatory
            1
        );

        if (task_created != pdPASS) {
            ESP_LOGE(TAG, "Failed to create SBUS task");
            return ESP_ERR_NO_MEM;
        }

        is_running_ = true;
        ESP_LOGI(TAG, "SBUS Started");
        return ESP_OK;
    }

    esp_err_t SBUS::stop() {
        if (is_running_) {
            return ESP_ERR_INVALID_STATE;
        }

        if (task_handle_ != nullptr) {
            vTaskDelete(task_handle_);
            task_handle_ = nullptr;
        }

        is_running_ = false;
        ESP_LOGI(TAG, "SBUS stopped");
        return ESP_OK;
    }

    void SBUS::sbusTask(void *parameters) {
        const auto instance = static_cast<SBUS *>(parameters);
        constexpr uint8_t start_byte = 0x0F;
        constexpr uint8_t end_byte = 0x00;
        uint8_t byte;
        TickType_t last_frame_time = xTaskGetTickCount();
        TickType_t last_wake_time = xTaskGetTickCount();

        ESP_LOGI(TAG, "sbus task started");

        uart_flush(instance->config_.uart_num);

        while (true) {
            // Wait for start byte
            // NOTE: pdMS_TO_TICKS(5) take note and see if this time can be optimized. (boring math)
            while (uart_read_bytes(instance->config_.uart_num, &byte, 1, pdMS_TO_TICKS(14)) > 0) {
                if (byte == start_byte) {
                    instance->frame_buffer_[0] = byte;
                    instance->buffer_index_ = 1;
                    break;
                }
            }

            if (instance->buffer_index_ == 1) {
                // Read the rest of the frame
                int remaining = FRAME_SIZE - 1;
                int read = uart_read_bytes(instance->config_.uart_num,
                                           &instance->frame_buffer_[1],
                                           remaining,
                                           pdMS_TO_TICKS(1));

                if (read == remaining && instance->frame_buffer_[FRAME_SIZE - 1] == end_byte) {
                    // Valid frame received
                    instance->processFrame(instance->frame_buffer_, FRAME_SIZE);

                    // Update timing
                    TickType_t current_time = xTaskGetTickCount();
                    float interval = static_cast<float>(current_time - last_frame_time) * portTICK_PERIOD_MS;
                    if (interval > 0) {
                        instance->current_data_.quality.frame_interval_ms = interval;
                    }
                    last_frame_time = current_time;
                }

                instance->buffer_index_ = 0;
            }

            // NOTE: debugging should place this all of the codebase t
            // TickType_t now = xTaskGetTickCount();
            // TickType_t delta_ticks = now - last_wake_time;
            //
            // // Avoid divide-by-zero
            // if (delta_ticks > 0) {
            //     uint32_t freq_hz = 1000 / delta_ticks;
            //     ESP_LOGI(TAG, "Frequency: %u Hz", freq_hz);
            // }

            // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(instance->config_.targetFreq));
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(7));

            // if (instance->log_freq_) { // TODO: connect log_freq_ to configmanager
                // static TickType_t prev_wake = 0;
                // TickType_t now = xTaskGetTickCount();
                // if (prev_wake != 0) {
                //     TickType_t delta_ticks = now - prev_wake;
                //     if (delta_ticks > 0) {
                //         uint32_t freq_hz = 1000 / delta_ticks; // assuming ticks are in ms
                //         ESP_LOGI(TAG, "Frequency: %lu Hz", freq_hz);
                //     }
                // }
                // prev_wake = now;
            // }
        }
    }

    void SBUS::processFrame(const uint8_t *frame, size_t len) {
        if (len != FRAME_SIZE) {
            ESP_LOGW(TAG, "invalid frame size %d", len);

            // std::ranges::fill(current_data_.channels_scaled, 1000);
            // current_data_.channels_scaled[static_cast<int>(SbusChannel::THROTTLE)] = 0;
            // VehicleData::instance().updateSBUS(current_data_);

            current_data_.quality.error_count++;
            return;
        }

        if (log_raw_) {
            ESP_LOGI(TAG, "Frame: [%02X %02X %02X %02X %02X %02X %02X %02X ...]",
                     frame[0], frame[1], frame[2], frame[3],
                     frame[4], frame[5], frame[6], frame[7]);
        }

        int byte_index = 1; // <- skip start byte
        int bit_index = 0;


        for (uint8_t ch = 0; ch < static_cast<uint8_t>(SbusChannel::CHANNEL_COUNT); ++ch) {
            uint16_t raw_value = 0;

            // Extract the 11 bits for channel
            for (int bit = 0; bit < 11; bit++) {
                if (frame[byte_index] & (1 << bit_index)) {
                    raw_value |= (1 << bit);
                }

                bit_index++;
                if (bit_index == 8) {
                    bit_index = 0;
                    byte_index++;
                }
            }

            current_data_.channels_raw[ch] = raw_value;
            current_data_.channels_scaled[ch] = rawToScaled(raw_value);

            // TODO: connect this to config manager and so you can choose channels to log
        }

        if (logging_) {
            ESP_LOGI(TAG, "CH: [%d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d]",
                     current_data_.channels_scaled[0], current_data_.channels_scaled[1],
                     current_data_.channels_scaled[2], current_data_.channels_scaled[3],
                     current_data_.channels_scaled[4], current_data_.channels_scaled[5],
                     current_data_.channels_scaled[6], current_data_.channels_scaled[7],
                     current_data_.channels_scaled[8], current_data_.channels_scaled[9],
                     current_data_.channels_scaled[10], current_data_.channels_scaled[11],
                     current_data_.channels_scaled[12], current_data_.channels_scaled[13],
                     current_data_.channels_scaled[14], current_data_.channels_scaled[15]);
        }

        monitorSignalQuality();
        VehicleData::instance().updateSBUS(current_data_);
    }


    void SBUS::monitorSignalQuality() {
        static constexpr float NOMINAL_FRAME_INTERVAL = 10.0f; // SBUS runs at ~70Hz (14ms)
        static constexpr float FRAME_INTERVAL_TOLERANCE = 10.0f; // ±5ms

        bool timing_ok = abs(current_data_.quality.frame_interval_ms - NOMINAL_FRAME_INTERVAL) <
                         FRAME_INTERVAL_TOLERANCE;

        current_data_.quality.valid_signal = timing_ok;

        if (!timing_ok) {
            ESP_LOGW(TAG, "Signal quality: %s, Frame interval: %.2fms",
                     timing_ok ? "OK" : "BAD", current_data_.quality.frame_interval_ms);
        }


        static constexpr int WINDOW_SIZE = 5; // TODO: evaluate if this should be moved to menuconfig
        static int good_frames = 0;

        // if (timing_ok) good_frames++;
        // if (good_frames > WINDOW_SIZE) good_frames = WINDOW_SIZE;

        if (timing_ok) std::min(good_frames++, WINDOW_SIZE);

        current_data_.quality.frame_loss_percent = static_cast<uint8_t>(100 - (good_frames * 100 / WINDOW_SIZE));
    }
}
