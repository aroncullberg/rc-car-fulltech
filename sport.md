# calculating checksum
header + data id + data value + checksum = 0xFF
checksum = 0xFF - (sum & 0xFF)

# Physical ID's
how to get sensor id from hex
```c++
sensor_id = (raw_byte & 0x1F) + 1;
```

```text
01 - 0x00 - Vario2 (altimeter high precision)
02 - 0xA1 - FLVSS LiPo sensor
03 - 0x22 - FAS-40S Current sensor
04 - 0x83 - GPS/Altimeter (normal percision)
05 - 0xE4 - RPM
06 - 0x45 - SPUART(Host)
07 - 0xC6 - SPUART(Remote)
08 - 0x67 -
09 - 0x48 -
10 - 0xE9 -
11 - 0x6A -
12 - 0xCB -
13 - 0xAC -
14 - 0x0D -
15 - 0x8E -
16 - 0x2F -
17 - 0xD0 -
18 - 0x71 -
19 - 0xF2 -
20 - 0x53 -
21 - 0x34 -
22 - 0x95 -
23 - 0x16 -
24 - 0xB7 -
25 - 0x98 -
26 - 0x39 -
27 - 0xBA -
28 - 0x1B -
```

// telemtry frame:
// byte   1 (1 bit) : - data frame header   (0x10)
// byte 2-3 (2 bits) : - data ID             (16 bit value type identifer
// byte 4-7 (4 bits) : - data value          (LSB first)
// byte   8 (1 bit)  : - checksum            (sum of bytes need to equal 0xFF)

```c++
constexpr uart_port_t uart_num = UART_NUM_1;
    constexpr gpio_num_t gpio = GPIO_NUM_17;

    // note: does order matter here?
    // gpio as open-drain input + output
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
    gpio_set_direction(gpio, GPIO_MODE_INPUT_OUTPUT_OD);

    // S.port is 57600 8N1
    uart_config_t uart_config{
        .baud_rate = 57'600,
        .data_bits = UART_DATA_8_BITS, // 8
        .parity = UART_PARITY_DISABLE, // N
        .stop_bits = UART_STOP_BITS_1, // 1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, gpio, gpio, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_line_inverse(uart_num, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    uart_driver_install(uart_num, 256, 256, 0, nullptr, 0);
    uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX);
    
    uint8_t byte;

    uint64_t last_time_us{};

while (1) {
        uint64_t delta_us{};

        int len = uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            ESP_LOGE(TAG, "Failed to read byte from UART");
            continue;
        }

        if (byte != 0x7E) {
            ESP_LOGW(TAG, "Invalid start byte: %02X", byte);
            continue;
        }

        uint64_t current_time_us = esp_timer_get_time();
        if (last_time_us > 0) {
            delta_us = current_time_us - last_time_us;
        }
        last_time_us = current_time_us;

        uint8_t sensor_id;
        len = uart_read_bytes(uart_num, &sensor_id, 1, pdMS_TO_TICKS(100));
        if (len <= 0) {
            ESP_LOGE(TAG, "Failed to read sensor ID from UART");
            continue;
        }

        ESP_LOGD(TAG, "%02X %02X (%llu ms)", byte, sensor_id, delta_us / 1000ULL);

        if (sensor_id != 0xE4) {
            continue;
        }

        // ESP_LOGI(TAG, "%02X %02X (%llu ms)", byte, sensor_id, delta_us / 1000ULL);
        send_telemetry_frame();
}
```

```c++
void send_telemetry_frame()
{
    uint16_t data_id = 0x0500; // FSSP_DATAID_RPM
    int32_t data_value = 50000;
    uint8_t frame[8]{};

    frame[0] = 0x10;
    frame[1] = data_id & 0xFF; // LSB 0x0500 & 0xFF -> 0x00
    frame[2] = data_id >> 8;   // MSB 0x0500 >> 8   -> 0x05
    frame[3] = data_value & 0xFF; //    1234 -> 0x04D2 -> 0x04D2 & 0xFF -> 0xD2
    frame[4] = (data_value >> 8) & 0xFF; // 1234 -> 0x04D2 -> (0x04D2 >> 8) & 0xFF -> 0x04
    frame[5] = (data_value >> 16) & 0xFF;
    frame[6] = (data_value >> 24) & 0xFF;
    frame[7] = checksum(frame, sizeof(frame));

    uart_write_bytes(UART_NUM_1, frame, sizeof(frame));
    uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(20));
}
```

```c++
uint8_t checksum(const uint8_t* packet, int len)
{
    uint16_t sum = 0;
    for (int i = 0; i < len; i++) {
        sum += packet[i];
        sum += sum >> 8;
        sum &= 0xFF;
    }
    return 0xFF - static_cast<uint8_t>(sum);
}
```