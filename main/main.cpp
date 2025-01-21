#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sbus.h"
#include "esp_log.h"
#include "DShotRMT.h"
#include <algorithm>

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

// Calculate target throttle (20%)
static const uint16_t TARGET_THROTTLE = THROTTLE_MIN + ((THROTTLE_MAX - THROTTLE_MIN) * TARGET_THROTTLE_PERCENT / 100);

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

void motor_control_task(void *pvParameters) {
    // Create and initialize DShot driver
    DShotRMT dshot(MOTOR_PIN, DSHOT600_BIDIRECTIONAL);
    
    ESP_LOGI(TAG, "Initializing DShot...");
    dshot.begin();
    
    ESP_LOGI(TAG, "Armed. Starting throttle ramp...");
    ESP_LOGI(TAG, "Target throttle: %d", TARGET_THROTTLE);
    
    uint16_t current_throttle = THROTTLE_MIN;
    uint32_t erpm;
    float rpm_ratio = DShotRMT::getErpmToRpmRatio(POLES);
    
    while (1) {
        // Send current throttle value
        dshot.sendThrottle(current_throttle);
        
        // Wait for and read telemetry
        if (dshot.waitForErpm(erpm) == ESP_OK) {
            if (erpm != 0xFFFF) { // Valid telemetry received
                float rpm = erpm * rpm_ratio;
                ESP_LOGI(TAG, "Throttle: %d, eRPM: %lu, RPM: %.2f", 
                        current_throttle, erpm, rpm);
            }
        }
        
        // Increase throttle if not at target
        if (current_throttle < TARGET_THROTTLE) {
            current_throttle = std::min(static_cast<uint16_t>(current_throttle + 10), static_cast<uint16_t>(TARGET_THROTTLE));
        }
        
        vTaskDelay(pdMS_TO_TICKS(RAMP_STEP_MS));
    }
}


extern "C" void app_main(void) {
    ESP_LOGI(TAG, "Starting SBUS example");

    // Initialize SBUS
    sbus = new SBUS(SBUS_UART);
    esp_err_t ret = sbus->begin(SBUS_RX_PIN, SBUS_TX_PIN, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SBUS");
        return;
    }

    // Increased stack size to 4096
    //xTaskCreate(sbus_read_task, "sbus_read", 4096, sbus, 5, NULL);

    ESP_LOGI(TAG, "SBUS initialization complete");

    ESP_LOGI(TAG, "Starting DShot test application");
    
    // Create motor control task
    xTaskCreate(motor_control_task, "motor_ctl", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Task created, system running");

}