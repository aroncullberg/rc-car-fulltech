#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sbus.h"
#include "esp_log.h"
#include "dshot_esc_encoder.h"
#include <algorithm>
#include "driver/rmt_tx.h"

#ifndef TAG
#define TAG "main"
#endif

#define SBUS_RX_PIN GPIO_NUM_18
#define SBUS_TX_PIN GPIO_NUM_5
#define SBUS_UART UART_NUM_1


#define MOTOR_PIN GPIO_NUM_7
#define THROTTLE_MIN 48
#define THROTTLE_MAX 2047
#define TARGET_THROTTLE_PERCENT 20
#define RAMP_STEP_MS 100  // Time between throttle increases
#define POLES 14  // Typical for many brushless motors, adjust if different
#define DSHOT_ESC_RESOLUTION_HZ 40000000 // 40MHz resolution, DSHot protocol needs a relative high resolution
#define DSHOT_ESC_GPIO_NUM      7


// Global objects
SBUS *sbus = nullptr;

void sbus_read_task(void *pvParameters) {
    SBUS *sbus = (SBUS *)pvParameters;
    uint16_t channels[16];
    char log_buffer[128]; // Pre-allocate buffer for logging

    while (1) {
        if (sbus->read(channels)) {
            // Log all channels in a single message to reduce stack usage
            snprintf(log_buffer, sizeof(log_buffer), 
                    "CH: %4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d,%4d",
                    channels[0], channels[1], channels[2], channels[3],
                    channels[4], channels[5], channels[6], channels[7],
                    channels[8], channels[9], channels[10], channels[11],
                    channels[12], channels[13], channels[14], channels[15]);
            ESP_LOGI(TAG, "%s", log_buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Increased delay slightly
    }
}


extern "C" void app_main(void) {
 ESP_LOGI(TAG, "Create RMT TX channel");
    rmt_channel_handle_t esc_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = static_cast<gpio_num_t>(DSHOT_ESC_GPIO_NUM),
        .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
        .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
        .mem_block_symbols = 64,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_chan));

    ESP_LOGI(TAG, "Install Dshot ESC encoder");
    rmt_encoder_handle_t dshot_encoder = NULL;
    dshot_esc_encoder_config_t encoder_config = {
        .resolution = DSHOT_ESC_RESOLUTION_HZ,
        .baud_rate = 300000, // DSHOT300 protocol
        .post_delay_us = 50, // extra delay between each frame
    };
    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(esc_chan));

    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // infinite loop
    };
    dshot_esc_throttle_t throttle = {
        .throttle = 0,
        .telemetry_req = false, // telemetry is not supported in this example
    };

    ESP_LOGI(TAG, "Start ESC by sending zero throttle for a while...");
    ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Increase throttle, no telemetry");
    for (uint16_t thro = 100; thro < 1000; thro += 10) {
        throttle.throttle = thro;
        ESP_ERROR_CHECK(rmt_transmit(esc_chan, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
        // the previous loop transfer is till undergoing, we need to stop it and restart,
        // so that the new throttle can be updated on the output
        ESP_ERROR_CHECK(rmt_disable(esc_chan));
        ESP_ERROR_CHECK(rmt_enable(esc_chan));
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}