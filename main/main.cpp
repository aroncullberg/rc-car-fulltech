#include <esp_console.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <stdio.h>

#include <rom/ets_sys.h>

#include "esp_vfs.h"

#define MAVLINK_USE_MESSAGE_INFO
// #include "c_library_v2/common/mavlink.h"
#include "c_library_v2/ardupilotmega/mavlink.h"

#include <sys/param.h>
#include <sys/socket.h>

#include "esp-libtelnet.h"

#ifndef TAG
#define TAG "main"
#endif


#include <string.h>
#include <driver/uart.h>
#include <hal/uart_types.h>
#include <soc/gpio_num.h>25

static const uint8_t SYS_ID = 1; // pick anything ≠ 0
static const uint8_t COMP_ID = MAV_COMP_ID_AUTOPILOT1;
static volatile bool armed = false;
static volatile mavlink_radio_status_t radio_status{};
static volatile int32_t lat = 594048890; // 59.329300 °  → deg E7
static volatile int32_t lon = 135811060; // 18.068600 °

extern "C" void vApplicationStackOverflowHook(TaskHandle_t t, char* name)
{
    ets_printf("\n*** STACK OVERFLOW: %s ***\n", name);
    abort(); // or esp_restart();
}

static inline void tx_mav(const mavlink_message_t& m)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    const uint16_t len = mavlink_msg_to_send_buffer(buf, &m);
    uart_write_bytes(UART_NUM_1, buf, len);
}


static void publish_version(void)
{
    // --- AUTOPILOT_VERSION (id 148) ---
    mavlink_autopilot_version_t av{};
    av.capabilities =
        MAV_PROTOCOL_CAPABILITY_MAVLINK2 | // <-- tell QGC you do v2
        MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT |
        MAV_PROTOCOL_CAPABILITY_COMMAND_INT |
        MAV_PROTOCOL_CAPABILITY_MISSION_INT;
    av.flight_sw_version =
        (1u << 24) | (0 << 16) | (0 << 8) | FIRMWARE_VERSION_TYPE_DEV;
    av.vendor_id = 4242;
    av.product_id = 1;
    av.uid = 0x1122334455667788ULL;

    mavlink_message_t m;
    mavlink_msg_autopilot_version_encode(SYS_ID, COMP_ID, &m, &av);
    tx_mav(m);

    // --- PROTOCOL_VERSION (id 300) ---
    mavlink_protocol_version_t pv{};
    pv.min_version = 0x200; // “2.0”
    pv.max_version = 0x200;
    mavlink_msg_protocol_version_encode(SYS_ID, COMP_ID, &m, &pv);
    tx_mav(m);
}

static void rx_task(void*)
{
    mavlink_message_t msg;
    mavlink_status_t status{};

    uint8_t byte;
    while (true) {
        if (uart_read_bytes(UART_NUM_1, &byte, 1, portMAX_DELAY) == 0) continue;

        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            switch (msg.msgid) {
            case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
                // mavlink_rc_channels_override_t rc;
                // mavlink_msg_rc_channels_override_decode(&msg, &rc);
                //
                // if (static size_t c = 0; c++ % 100 == 0) {
                //
                //     ESP_LOGI(TAG, "RC override: %d %d %d %d %d %d %d %d",
                //              rc.chan1_raw, rc.chan2_raw, rc.chan3_raw,
                //              rc.chan4_raw, rc.chan5_raw, rc.chan6_raw,
                //              rc.chan7_raw, rc.chan8_raw);
                // }
                break;
            case MAVLINK_MSG_ID_RADIO_STATUS:
                mavlink_radio_status_t radio;
                mavlink_msg_radio_status_decode(&msg, &radio);

                radio_status.rssi = radio.rssi;
                radio_status.remrssi = radio.remrssi;
                radio_status.txbuf = radio.txbuf;
                radio_status.noise = radio.noise;
                radio_status.remnoise = radio.remnoise;
                radio_status.rxerrors = radio.rxerrors;
                radio_status.fixed = radio.fixed;
                break;
            case MAVLINK_MSG_ID_HEARTBEAT:
                break;
            case MAVLINK_MSG_ID_COMMAND_LONG:
                {
                    mavlink_command_long_t c;
                    mavlink_msg_command_long_decode(&msg, &c);
                    if (c.command == MAV_CMD_REQUEST_MESSAGE) {
                        switch (static_cast<size_t>(c.param1)) {
                        case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
                            ESP_LOGI(TAG, "MAV_CMD_REQUEST_MESSAGE -> Responded");
                            publish_version();
                        default:
                            ESP_LOGW(TAG, "MAV_CMD_REQUEST_MESSAGE with ID %f", c.param1);
                        }
                    }
                    else if (c.command == MAV_CMD_COMPONENT_ARM_DISARM &&
                        c.target_system == SYS_ID &&
                        c.target_component == COMP_ID) {
                        armed = (c.param1 > 0.5f);

                        mavlink_message_t ack;
                        mavlink_msg_command_ack_pack_chan(
                            SYS_ID, COMP_ID, MAVLINK_COMM_0, &ack,
                            MAV_CMD_COMPONENT_ARM_DISARM,
                            MAV_RESULT_ACCEPTED, 0, 0, 0, 0);
                        tx_mav(ack);

                        ESP_LOGI(TAG, "Arm state -> %s",
                                 armed ? "ARMED" : "DISARMED");
                    }
                    else {
                        ESP_LOGW(TAG, "Command: %d (param1: %d)", c.command, static_cast<int>(c.param1));
                    }
                }
                break;
            default:
                const mavlink_message_info_t* msg_info = mavlink_get_message_info_by_id(msg.msgid);
                ESP_LOGW(TAG, "%s", msg_info->name);
            }
        }
    }
}

static void debug(void*)
{
    mavlink_message_t hb;
    uint8_t base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED |
        MAV_MODE_FLAG_STABILIZE_ENABLED |
        (armed ? MAV_MODE_FLAG_SAFETY_ARMED : 0);

    mavlink_msg_heartbeat_pack_chan(
        SYS_ID, COMP_ID, MAVLINK_COMM_0, &hb,
        MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
        // MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_GENERIC,
        base_mode,
        0, // custom_mode
        armed ? MAV_STATE_ACTIVE : MAV_STATE_STANDBY);
    tx_mav(hb);

    // --------------------------------------------------------------

    const uint32_t ok = MAV_SYS_STATUS_SENSOR_3D_GYRO |
        MAV_SYS_STATUS_SENSOR_3D_ACCEL |
        MAV_SYS_STATUS_SENSOR_GPS;

    mavlink_message_t st;
    mavlink_msg_sys_status_pack_chan(
        SYS_ID, COMP_ID, MAVLINK_COMM_0, &st,
        ok, /* present                 */
        ok, /* enabled                 */
        ok, /* health                  */
        50, /* CPU load ‰              */
        UINT16_MAX, /* voltage_battery         */
        -1, /* current_battery (cA)    */
        -1, /* battery_remaining (%)   */
        0, 0, 0, 0, 0, 0, /* drop & error counters */
        ok, /* present_extended        */
        ok, /* enabled_extended        */
        ok); /* health_extended         */
    tx_mav(st);

    // --------------------------------------------------------------

    mavlink_message_t rt;
    mavlink_radio_status_t radio_status_local{};
    radio_status_local.rssi = radio_status.rssi;
    radio_status_local.remrssi = radio_status.remrssi;
    radio_status_local.txbuf = radio_status.txbuf;
    radio_status_local.noise = radio_status.noise;
    radio_status_local.remnoise = radio_status.remnoise;
    radio_status_local.rxerrors = radio_status.rxerrors;
    radio_status_local.fixed = radio_status.fixed;
    mavlink_msg_radio_status_encode_chan(SYS_ID, MAV_COMP_ID_TELEMETRY_RADIO, MAVLINK_COMM_0, &rt, &radio_status_local);
    tx_mav(rt);

    // --------------------------------------------------------------

    mavlink_esc_telemetry_1_to_4_t esc{};
    static int voltage = 12000;
    if (voltage > 3000) {
        voltage -= 100; // simulate battery drain
    }
    else {
        voltage = 12000; // reset to 12.0 V
    }

    esc.voltage[0] = voltage;
    esc.voltage[1] = voltage + 1000;
    esc.voltage[2] = voltage + 500;
    esc.voltage[3] = voltage - 500;

    esc.current[0] = 5000; // 5.0 A

    esc.temperature[0] = 300; // 30.0 °C

    // randomzie RPMs a bit
    esc.rpm[0] = 10000 + (esp_random() % 1000);
    esc.rpm[1] = 10000 + (esp_random() % 1000);
    esc.rpm[2] = 10000 + (esp_random() % 1000);
    esc.rpm[3] = 10000 + (esp_random() % 1000);

    mavlink_message_t et;
    mavlink_msg_esc_telemetry_1_to_4_encode_chan(
        SYS_ID, // keep same IDs as heartbeat
        COMP_ID,
        MAVLINK_COMM_0,
        &et, &esc);

    tx_mav(et);

    // --------------------------------------------------------------

    mavlink_gps_raw_int_t gps{};
    gps.time_usec = esp_timer_get_time() * 1000; // convert to µs
    gps.lat = lat; // 59.329300 ° → deg E7
    gps.lon = lon; // 18.068600 ° → deg E7
    gps.alt = 1000; // 100.0 m
    gps.eph = UINT16_MAX;
    gps.epv = UINT16_MAX; // no vertical accuracy
    gps.vel = 200; // 10.0 m/s
    gps.cog = 0; // 0° (North)
    gps.fix_type = GPS_FIX_TYPE_3D_FIX;
    gps.satellites_visible = 8; // 8 satellites
    gps.alt_ellipsoid = 1000; // 100.0 m
    gps.h_acc = 100; // 10.0 m
    gps.v_acc = 100; // 10.0 m
    gps.vel_acc = 100; // 10.0 m/s
    gps.hdg_acc = 100; // 10.0° (degrees)
    gps.yaw = 0; // 0° (North)

    mavlink_message_t gt{};
    mavlink_msg_gps_raw_int_encode_chan(SYS_ID, COMP_ID, MAVLINK_COMM_0,
                                                &gt, &gps);
    tx_mav(gt);

    mavlink_vfr_hud_t vfr_hud;
    vfr_hud.airspeed = 100; // 10.0 m/s
    vfr_hud.groundspeed = 100; // 10.0 m/s
    vfr_hud.heading = 0; // 0° (North)
    vfr_hud.throttle = 50; // 50% throttle
    vfr_hud.alt = 1000; // 100.0 m
    vfr_hud.climb = 0; // 0.0 m/s (no climb)

    mavlink_message_t vt{};
    mavlink_msg_vfr_hud_encode_chan(SYS_ID, COMP_ID, MAVLINK_COMM_0,
                                    &vt, &vfr_hud);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    tx_mav(vt);
    // --------------------------------------------------------------

    mavlink_scaled_imu_t scaled_imu;
    scaled_imu.time_boot_ms = esp_timer_get_time();
    scaled_imu.xacc = 0; // 0.0 m/s²
    scaled_imu.yacc = 0; // 0.0 m/s²
    scaled_imu.zacc = 0; // 0.0 m/s²
    scaled_imu.xgyro = 0; // 0.0 rad/s
    scaled_imu.ygyro = 0; // 0.0 rad/s
    scaled_imu.zgyro = 0;
    scaled_imu.xmag = 0; // 0.0 mG
    scaled_imu.ymag = 0; // 0.0 mG
    scaled_imu.zmag = 0; // 0.0 mG
    scaled_imu.temperature = 250; // 25.0 °C

    mavlink_message_t sm{};
    mavlink_msg_scaled_imu_encode_chan(SYS_ID, COMP_ID, MAVLINK_COMM_0,
                                       &sm, &scaled_imu);
    tx_mav(sm);
}


extern "C" void app_main(void)
{
    const uart_port_t uart_num = UART_NUM_1;
    int rx_pin = 18;
    int tx_pin = 17;

    uart_config_t cfg = {
        .baud_rate = 460800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num,
        /*rx_buffer_size*/ 1024,
        /*tx_buffer_size*/ 0,
        /*event_queue_len*/ 0,
        /*event_queue*/ nullptr,
        /*intr_alloc_flags*/ 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_pin, rx_pin,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));


    esp_timer_handle_t debug_timer;
    const esp_timer_create_args_t debug_args = {
        .callback = debug, .name = "debug"
    };
    ESP_ERROR_CHECK(esp_timer_create(&debug_args, &debug_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(debug_timer, 1'000'000)); // µs

    xTaskCreatePinnedToCore(rx_task, "rx_task", 4096, nullptr, 5, nullptr, 0);
}
