#include "dshot_controller.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "DShot_Test";

extern "C" void app_main(void) {
    // Create DShot controller on GPIO 18
    DShotController dshot(GPIO_NUM_18);
    
    // Initialize the controller
    ESP_ERROR_CHECK(dshot.init());
    
    // Arm the ESC
    ESP_ERROR_CHECK(dshot.arm());
    ESP_LOGI(TAG, "ESC Armed");
    
    // Wait a bit after arming
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Simple throttle test
    ESP_LOGI(TAG, "Starting throttle test");
    
    // Gradually increase throttle
    for (uint16_t throttle = 48; throttle < 200; throttle += 10) {
        ESP_LOGI(TAG, "Setting throttle to %d", throttle);
        ESP_ERROR_CHECK(dshot.send_throttle(throttle));
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    
    // Return to zero
    ESP_ERROR_CHECK(dshot.send_throttle(0));
    ESP_LOGI(TAG, "Test complete");
}