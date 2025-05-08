#include <esp_console.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>
#include <cstring>
#include <dashboard_http_server.hpp>
#include <esp_http_server.h>

#include "receiver.h"
#include "sbus_driver.h"
#include "gps.h"
#include "nmea_driver.h"
#include "imu.hpp"
#include "icm_20948_driver.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"

#include <string>
#include <task_stat_driver.hpp>
#include <unordered_map>
#include <rom/ets_sys.h>
#include <rom/rom_layout.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "json_config.h"

#include "esp_heap_task_info.h"
#include "esp_heap_caps.h"

#include "esp_vfs.h"
#include "esp_littlefs.h"

#include "task_stats.hpp"


#include "esp_check.h"

#define MAX_TASKS 32
#define MAX_HEAP_TASKS 32

#ifndef TAG
#define TAG "main"
#endif

#define AP_SSID       "eduroam but safe"
#define AP_PASS       "supersecret"
#define AP_CHANNEL    1
#define AP_MAX_CONN   4
#define AP_AUTHMODE   WIFI_AUTH_WPA3_PSK

char prompt[100]; // Prompt to be printed before each line

auto cmpHandle = [](auto const& a, auto const& b)
{
    return a.xHandle < b.xHandle;
};


extern "C" void vApplicationStackOverflowHook(TaskHandle_t t, char *name)
{
    ets_printf("\n*** STACK OVERFLOW: %s ***\n", name);
    abort();                 // or esp_restart();
}

extern "C" void app_main(void)
{
    // ------------------------------------------------------------------
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    wifi_config.ap = {
        .ssid = AP_SSID,
        .password = AP_PASS,
        .channel = AP_CHANNEL,
        .authmode = AP_AUTHMODE,
        .max_connection = AP_MAX_CONN,
        .beacon_interval = 100
    };

    if (strlen(AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 7) Boost to max TX power (0.25 dBm units; 82×0.25 = 20.5 dBm)
    // ESP_ERROR_CHECK( esp_wifi_set_max_tx_power(82) );

    ESP_LOGI("wifi_ap", "Soft-AP up: SSID=%s  PASS=%s  CH=%d",
             AP_SSID, AP_PASS, AP_CHANNEL);

    // ------------------------------------------------------------------
    esp_err_t err = config::JsonConfig::instance().load();
    if (err != ESP_OK) {
        ESP_LOGW("app", "config load failed: %s", esp_err_to_name(err));
    }

    config::JsonConfig::instance().attach_to_console();

    // ------------------------------------------------------------------

    esp_console_repl_t* usb_repl = nullptr;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    repl_config.prompt = LOG_ANSI_COLOR_BOLD(LOG_ANSI_COLOR_GREEN) "esp > " LOG_ANSI_COLOR_RESET;
    repl_config.max_cmdline_length = 256;
    repl_config.task_core_id = 0;

    esp_console_register_help_command();

    esp_console_dev_usb_serial_jtag_config_t jtag_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&jtag_config, &repl_config, &usb_repl));
    ESP_ERROR_CHECK(esp_console_start_repl(usb_repl));

    // ------------------------------------------------------------------

    dashboard::HttpServer::instance().start();

    // ------------------------------------------------------------------

    // proto::SbusDriver::Config sbus_config;
    // sbus_config.uart_num = UART_NUM_1;
    // sbus_config.uart_tx_pin = GPIO_NUM_17;
    // sbus_config.uart_tx_pin = GPIO_NUM_NC;
    // sbus_config.uart_rx_pin = GPIO_NUM_18;
    // sbus_config.buad_rate = 100'000;
    // static proto::SbusDriver sbus_driver(sbus_config);
    // ESP_ERROR_CHECK(sbus_driver.init());
    // ESP_ERROR_CHECK(sbus_driver.start());
    //
    // proto::NmeaDriver::Config gps_config;
    // gps_config.uart_num = UART_NUM_2;
    // gps_config.uart_tx_pin = GPIO_NUM_7;
    // gps_config.uart_rx_pin = GPIO_NUM_15;
    // gps_config.buad_rate = 38'400;
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
    imu_config.spi_clock_hz = 20 * 1000 * 1000;
    imu_config.int_gpio = GPIO_NUM_8;
    imu_config.task_stack = 8192;
    imu_config.task_prio = 5;
    imu_config.queue_len = 16;
    static proto::Icm20948Driver imu_driver(imu_config);
    ESP_ERROR_CHECK(imu_driver.init());
    ESP_ERROR_CHECK(imu_driver.start());

    // ------------------------------------------------------------------

    proto::TaskStatDriver::Config task_stat_config;
    task_stat_config.period = pdMS_TO_TICKS(5000);
    task_stat_config.task_stack = 4096;
    static proto::TaskStatDriver task_stat_driver(task_stat_config);
    ESP_ERROR_CHECK(task_stat_driver.init());
    ESP_ERROR_CHECK(task_stat_driver.start());

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(250));

        cJSON* msg = cJSON_CreateObject();

        {
            cJSON* j_imu = cJSON_CreateObject();

            cJSON* j_imu_accel = cJSON_CreateObject();
            cJSON_AddNumberToObject(j_imu_accel, "x", motion::Imu::instance().get_accel().x  );
            cJSON_AddNumberToObject(j_imu_accel, "y", motion::Imu::instance().get_accel().y  );
            cJSON_AddNumberToObject(j_imu_accel, "z", motion::Imu::instance().get_accel().z  );
            cJSON_AddNumberToObject(j_imu_accel, "frequency", motion::Imu::instance().get_accel().frequency);
            cJSON_AddItemToObject(j_imu, "accel", j_imu_accel);

            cJSON* j_imu_gyro = cJSON_CreateObject();
            cJSON_AddNumberToObject(j_imu_gyro, "x", motion::Imu::instance().get_gyro().x  );
            cJSON_AddNumberToObject(j_imu_gyro, "y", motion::Imu::instance().get_gyro().y  );
            cJSON_AddNumberToObject(j_imu_gyro, "z", motion::Imu::instance().get_gyro().z  );
            cJSON_AddNumberToObject(j_imu_gyro, "frequency", motion::Imu::instance().get_gyro().frequency);
            cJSON_AddItemToObject(j_imu, "gyro", j_imu_gyro);

            cJSON_AddItemToObject(msg, "imu", j_imu);
        }


        {
            auto loc = nav::Gps::instance().getLocation();
            auto date = nav::Gps::instance().getDate();
            auto time = nav::Gps::instance().getTime();
            auto speed = nav::Gps::instance().getSpeed();
            auto course = nav::Gps::instance().getCourse();
            auto alt = nav::Gps::instance().getAltitude();
            auto sats = nav::Gps::instance().getSatellite();
            auto hdop = nav::Gps::instance().getHDOP();

            cJSON* gps = cJSON_CreateObject();

            // location
            cJSON* jloc = cJSON_CreateObject();
            cJSON_AddBoolToObject(jloc, "valid", loc.isValid);
            cJSON_AddNumberToObject(jloc, "lat", loc.lat_e7 / 1e7f);
            cJSON_AddNumberToObject(jloc, "lon", loc.lon_e7 / 1e7f);
            cJSON_AddNumberToObject(jloc, "age_ms", loc.age);
            cJSON_AddItemToObject(gps, "location", jloc);

            // date
            cJSON* jdate = cJSON_CreateObject();
            cJSON_AddBoolToObject(jdate, "valid", date.isValid);
            cJSON_AddNumberToObject(jdate, "year", date.year);
            cJSON_AddNumberToObject(jdate, "month", date.month);
            cJSON_AddNumberToObject(jdate, "day", date.day);
            cJSON_AddNumberToObject(jdate, "age_ms", date.age);
            cJSON_AddItemToObject(gps, "date", jdate);

            // time
            cJSON* jtime = cJSON_CreateObject();
            cJSON_AddBoolToObject(jtime, "valid", time.isValid);
            cJSON_AddNumberToObject(jtime, "hour", time.hour);
            cJSON_AddNumberToObject(jtime, "minute", time.minute);
            cJSON_AddNumberToObject(jtime, "second", time.second);
            cJSON_AddNumberToObject(jtime, "cs", time.centisecond);
            cJSON_AddNumberToObject(jtime, "age_ms", time.age);
            cJSON_AddItemToObject(gps, "time", jtime);

            // speed
            cJSON* jspeed = cJSON_CreateObject();
            cJSON_AddNumberToObject(jspeed, "mmps", speed.speed_mmps);
            cJSON_AddNumberToObject(jspeed, "kmph", speed.speed_kmph);
            cJSON_AddItemToObject(gps, "speed", jspeed);

            // course
            cJSON* jcourse = cJSON_CreateObject();
            cJSON_AddNumberToObject(jcourse, "centi_deg", course.course_cd);
            cJSON_AddItemToObject(gps, "course", jcourse);

            // altitude
            cJSON* jalt = cJSON_CreateObject();
            cJSON_AddNumberToObject(jalt, "m", alt.altitude_m);
            cJSON_AddItemToObject(gps, "altitude", jalt);

            // satellites
            cJSON* jsats = cJSON_CreateObject();
            cJSON_AddBoolToObject(jsats, "valid", sats.isValid);
            cJSON_AddNumberToObject(jsats, "count", sats.satellites);
            cJSON_AddNumberToObject(jsats, "age_ms", sats.age);
            cJSON_AddItemToObject(gps, "satellites", jsats);

            // hdop
            cJSON_AddNumberToObject(gps, "hdop", hdop);

            cJSON_AddItemToObject(msg, "gps", gps);
        }

        {
            auto values = rc::Receiver::instance().get_all();
            cJSON* rcobj = cJSON_CreateObject();
            cJSON* channels = cJSON_CreateArray();

            for (int i = 0; i < rc::k_channel_count; ++i) {
                int v = values[i];
                const auto& ch = rc::Receiver::instance().get(
                     static_cast<rc::ChannelIndex>(i));

                const char* state = ch.low()   ? "low"
                                   : ch.mid()   ? "mid"
                                   : ch.high()  ? "high"
                                   :              "unknown";

                cJSON* jch = cJSON_CreateObject();
                cJSON_AddNumberToObject(jch, "channel", i + 1);
                cJSON_AddNumberToObject(jch, "value",   v);
                cJSON_AddStringToObject(jch, "state",   state);

                cJSON_AddItemToArray(channels, jch);
            }

            cJSON_AddItemToObject(rcobj, "channels", channels);
            cJSON_AddItemToObject(msg, "rc", rcobj);
        }

        {
            cJSON* pid = cJSON_CreateObject();
            {
                cJSON* steering_pid = cJSON_CreateObject();

                cJSON_AddNumberToObject(steering_pid, "p", 0.0);
                cJSON_AddNumberToObject(steering_pid, "i", 0.0);
                cJSON_AddNumberToObject(steering_pid, "d", 0.0);
                cJSON_AddItemToObject(pid, "steering", steering_pid);
            }
            cJSON_AddItemToObject(msg, "pid", pid);
        }

        {
            cJSON* erpm = cJSON_CreateObject();
            cJSON_AddNumberToObject(erpm, "rpm_q1", 0.0);
            cJSON_AddNumberToObject(erpm, "rpm_q2", 0.0);
            cJSON_AddNumberToObject(erpm, "rpm_q3", 0.0);
            cJSON_AddNumberToObject(erpm, "rpm_q4", 0.0);
            cJSON_AddItemToObject(msg, "motor erpm", erpm);
        }

        {
            cJSON* j_tasks = cJSON_CreateArray();
            cJSON_AddNumberToObject(j_tasks, "count", diagnostics::TaskStats::instance().get().size());
            for (const auto& s : diagnostics::TaskStats::instance().get()) {
                if (!s.task_name || s.task_name[0] == '\0') continue;

                cJSON* task = cJSON_CreateObject();
                cJSON_AddStringToObject(task, "name", s.task_name);
                cJSON_AddNumberToObject(task, "runtime_share", s.runtime_share);
                cJSON_AddNumberToObject(task, "cpu_percent", s.cpu_percent);
                cJSON_AddStringToObject(task, "status", s.status);
                cJSON_AddNumberToObject(task, "high_water_mark", s.high_water_mark);
                cJSON_AddNumberToObject(task, "core", s.core);

                cJSON_AddItemToArray(j_tasks, task);
            }
            cJSON_AddItemToObject(msg, "tasks", j_tasks);
        }


        dashboard::HttpServer::instance().push(msg);
        // printf("pushed json to dashboard \n");
        cJSON_Delete(msg);
    }
}
