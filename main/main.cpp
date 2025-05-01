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
#include "icm_20948_driver.h"
#include "imu.h"


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

// Chatgpt
void printRunTimeStatsLight()
{
    // How many tasks are actually running?
    UBaseType_t uxArraySize = uxTaskGetNumberOfTasks();
    // Allocate an array of TaskStatus_t
    TaskStatus_t* pxTaskStatusArray = (TaskStatus_t*)pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));
    uint32_t ulTotalRunTime;

    // Fill in the array — this only does one pass internally, no sprintf()
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);

    // Now you have each pxTaskStatusArray[i].ulRunTimeCounter
    // Do your own percent math and lightweight printing
    for (UBaseType_t i = 0; i < uxArraySize; i++) {
        const char* name = pxTaskStatusArray[i].pcTaskName;
        uint32_t time = pxTaskStatusArray[i].ulRunTimeCounter;
        float pcnt = (ulTotalRunTime > 0)
                         ? (100.0f * static_cast<float>(time) / static_cast<float>(ulTotalRunTime))
                         : 0;
        printf("%-17s %8lu ticks   %5.1f%%\n", name, time, pcnt);
    }

    vPortFree(pxTaskStatusArray);
}

extern "C" [[noreturn]] void app_main(void)
{
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

    // proto::NmeaDriver::Config gps_config;
    // gps_config.uart_num = UART_NUM_2;
    // gps_config.uart_tx_pin = GPIO_NUM_7;
    // gps_config.uart_rx_pin = GPIO_NUM_15;
    // gps_config.buad_rate = 38400;
    // gps_config.rx_buffer_size = 2048;
    // gps_config.tx_buffer_size = 0;
    // gps_config.pattern_queue_size = 16;
    // gps_config.task_stack_size = 4096;
    // gps_config.task_priority = 5;
    // static proto::NmeaDriver gps_driver(gps_config);
    // ESP_ERROR_CHECK(gps_driver.init());
    // ESP_ERROR_CHECK(gps_driver.start());

    proto::Icm20948Driver::Config imu_config;
    imu_config.spi_host = SPI2_HOST;
    imu_config.spi_miso = GPIO_NUM_13;
    imu_config.spi_mosi = GPIO_NUM_11;
    imu_config.spi_sck = GPIO_NUM_12;
    imu_config.spi_cs = GPIO_NUM_10;
    imu_config.spi_clock_hz = 4 * 1000 * 1000;
    imu_config.int_gpio = GPIO_NUM_8;
    imu_config.task_stack = 4096;
    imu_config.task_prio = 5;
    static proto::Icm20948Driver imu_driver(imu_config);
    ESP_ERROR_CHECK(imu_driver.init());
    ESP_ERROR_CHECK(imu_driver.start());


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


        printf("\n=== GPS Status ===\n\n");

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

        printf("\n=== RC Status ===\n\n");

        auto values = rc::Receiver::instance().get_all();

        for (int i = 0; i < rc::k_channel_count; i++) {
            printf("CH%02d: %4d - ", i + 1, values[i]);
               printf("%s\n",
                     rc::Receiver::instance().get(static_cast<rc::ChannelIndex>(i)).low() ? "low" :
                     rc::Receiver::instance().get(static_cast<rc::ChannelIndex>(i)).mid() ? "mid" :
                     rc::Receiver::instance().get(static_cast<rc::ChannelIndex>(i)).high() ? "high" :
                        "unknown" );
        }


        printf("\n=== IMU Status ===\n\n");

        auto accel = motion::Imu::instance().getAccel();
        auto gyro = motion::Imu::instance().getGyro();
        auto quat6 = motion::Imu::instance().getQuat6();
        auto quat9 = motion::Imu::instance().getQuat9();
        auto stats = motion::Imu::instance().getStats();

        printf("Accel:  X: %6d  Y: %6d  Z: %6d\n",
               accel.x, accel.y, accel.z);
        printf("Gyro:   X: %6d  Y: %6d  Z: %6d\n", gyro.x, gyro.y, gyro.z);
        printf("Quat6:  X: %6ld  Y: %6ld  Z: %6ld\n", quat6.x, quat6.y, quat6.z);
        printf("Quat9:  X: %6ld  Y: %6ld  Z: %6ld  Accuracy: %6d\n",
               quat9.x, quat9.y, quat9.z, quat9.accuracy);
        printf("duty_cycle_permille: %f‰\n", static_cast<float>(stats.duty_cycle_permille) / 1000.0f);
        printf("average freq: %4.1f Hz\n", static_cast<float>(stats.average_freq_mill_hz) / 1000.0f);
        // printf("Runtime stats: \n%s\n", stats.run_time_stats);

        printRunTimeStatsLight();


        vTaskDelay(pdMS_TO_TICKS(1000));
    }

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
