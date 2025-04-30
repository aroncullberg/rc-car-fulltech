#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "log_monitor.h"
#include "config_manager.h"
#include <stdio.h>

#include "receiver.h"
#include "sbus_driver.h"
#include "gps.h"
#include "nmea_driver.h"


#ifndef TAG
#define TAG "main"
#endif

// // External PSRAM base address (?)
// #define EXTERNAL_PSRAM_BASE_ADDRESS 0x3F800000
//
// // Define the size of each heap region
// #define HEAP_REGION1_SIZE 0x10000  // 64 KB
// #define HEAP_REGION2_SIZE 0x20000  // 128 KB
//
// // Define the heap regions array
// static HeapRegion_t xHeapRegions[] =
// {
//     { (uint8_t *)EXTERNAL_PSRAM_BASE_ADDRESS, HEAP_REGION1_SIZE },
//     { (uint8_t *)EXTERNAL_PSRAM_BASE_ADDRESS + HEAP_REGION1_SIZE, HEAP_REGION2_SIZE },
//     { NULL, 0 } // Terminates the array
// };


extern "C" [[noreturn]] void app_main(void) {
    // ESP_LOGI("main", "Initializing ConfigManager");
    // esp_err_t ret = ConfigManager::instance().init();
    // if (ret != ESP_OK) {
    //     ESP_LOGE("main", "Failed to initialize ConfigManager: %d", ret);
    // } else {
    //     ESP_LOGI("main", "ConfigManager initialized successfully");
    //
    //     // Test direct access to a key
    //     bool imu_enabled = ConfigManager::instance().getBool("imu/enabled", true);
    //     ESP_LOGI("main", "Direct access to imu/enabled: %d", imu_enabled);
    // }

    // LogMonitor::Config log_config;
    // log_config.ap_ssid = "ESP32-Monitor";
    // log_config.ap_password = "password";
    //
    // LogMonitor::instance().init(log_config);
    // LogMonitor::instance().start();
    //
    // ESP_LOGI("main", "Log monitor started! Connect to WiFi SSID: %s", log_config.ap_ssid);
    // ESP_LOGI("main", "Use 'nc YOUR_ESP_IP 8888' to view logs");


    proto::SbusDriver::Config sbus_config;
    sbus_config.uart_num = UART_NUM_1;
    sbus_config.uart_tx_pin = GPIO_NUM_17;
    sbus_config.uart_rx_pin = GPIO_NUM_18;
    sbus_config.buad_rate = 100'000;
    static proto::SbusDriver sbus_driver(sbus_config);
    ESP_ERROR_CHECK(sbus_driver.init());
    ESP_ERROR_CHECK(sbus_driver.start());

    proto::NmeaDriver::Config gps_config;
    gps_config.uart_num = UART_NUM_2;
    gps_config.uart_tx_pin = GPIO_NUM_7;
    gps_config.uart_rx_pin = GPIO_NUM_15;
    gps_config.buad_rate = 38400;
    gps_config.rx_buffer_size = 2048;
    gps_config.tx_buffer_size = 0;
    gps_config.pattern_queue_size = 16;
    gps_config.task_stack_size = 4096;
    gps_config.task_priority = 5;
    static proto::NmeaDriver gps_driver(gps_config);
    ESP_ERROR_CHECK(gps_driver.init());
    ESP_ERROR_CHECK(gps_driver.start());

    while (true) {
           printf("\x1b[2J\x1b[H");

           auto loc = nav::Gps::instance().getLocation();
           auto date = nav::Gps::instance().getDate();
           auto time = nav::Gps::instance().getTime();
           auto speed = nav::Gps::instance().getSpeed();
           auto course = nav::Gps::instance().getCourse();
           auto alt = nav::Gps::instance().getAltitude();
           auto sats = nav::Gps::instance().getSatellite();
           auto hdop = nav::Gps::instance().getHDOP();


           printf("=== GPS Status ===\n\n");

           printf("Location : [%s]  Lat: %.7f  Lon: %.7f  Age: %4lums\n",
                  loc.isValid ? "✔" : "✘",
                  loc.lat_e7 / 1e7, loc.lon_e7 / 1e7,
                  loc.age);

           printf("Date     : [%s]  %04u-%02u-%02u  Age: %4lums\n",
                  date.isValid ? "✔" : "✘",
                  date.year, date.month, date.day,
                  date.age);

           printf("Time     : [%s]  %02u:%02u:%02u.%02u  Age: %4lums\n",
                  time.isValid ? "✔" : "✘",
                  time.hour, time.minute, time.second, time.centisecond,
                  time.age);

           printf("Speed    :      %-6ld mm/s  (%4ld km/h)\n",
                  speed.speed_mmps, speed.speed_kmph);

           printf("Course   :      %-6ld (%.2f°)\n",
                  course.course_cd, course.course_cd / 100.0);

           printf("Altitude :      %-6ld mm  (%.3f m)\n",
                  alt.altitude_m, alt.altitude_m / 1000.0);

           printf("Sats     : [%s]  Count: %2u  Age: %4lums\n",
                  sats.isValid ? "✔" : "✘",
                  static_cast<unsigned>(sats.satellites),
                  sats.age);

           printf("HDOP     :      %-u\n", hdop);

           printf("=== RC Status ===\n\n");

           auto values = rc::Receiver::instance().getAll();

           for (int i = 0; i < rc::kChannelCount; i++) {
                  printf("CH%02d: %4d  \n", i+1, values[i]);
           }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // TODO: either remove or change to use updated compoenents
    // #if CONFIG_IMU_ENABLE
    //     sensor::IMU::Config imu_config = {
    //         .spi_host = SPI2_HOST,
    //         .spi_miso_pin = CONFIG_IMU_SPI_MISO,
    //         .spi_mosi_pin = CONFIG_IMU_SPI_MOSI,
    //         .spi_sck_pin = CONFIG_IMU_SPI_CLK,
    //         .spi_cs_pin = CONFIG_IMU_SPI_CS,
    //         .targetFreq = Frequency::F100Hz
    //     };
    //     static sensor::IMU imu(imu_config);
    //     ESP_ERROR_CHECK(imu.init());
    //     ESP_ERROR_CHECK(imu.start());
    // #endif

    // Servo::Config servo_config;
    // servo_config.gpio_num = static_cast<gpio_num_t>(CONFIG_SERVO_OUTPUT_GPIO);
    // // servo_config.freq_hz = static_cast<uint32_t>(CONFIG_SERVO_FREQUENCY_HZ);
    // servo_config.min_pulse_width_us = static_cast<uint32_t>(1000);
    // servo_config.max_pulse_width_us = static_cast<uint32_t>(2000);
    //
    // VehicleDynamicsController::Config vd_config;
    // vd_config.motors_config.front_left_pin = static_cast<gpio_num_t>(38);
    // vd_config.motors_config.front_right_pin = static_cast<gpio_num_t>(39);
    // vd_config.motors_config.rear_left_pin = static_cast<gpio_num_t>(40);
    // vd_config.motors_config.rear_right_pin = static_cast<gpio_num_t>(41);
    // vd_config.motors_config.dshot_mode = DSHOT300_BIDIRECTIONAL;
    // vd_config.servo_config = servo_config;
    // vd_config.task_stack_size = 8162;
    // vd_config.task_priority = 7;
    // vd_config.frequency = Frequency::F200Hz;
    //
    // static VehicleDynamicsController vd_controller(vd_config);
    // ESP_ERROR_CHECK(vd_controller.init());
    // ESP_ERROR_CHECK(vd_controller.start());
}
