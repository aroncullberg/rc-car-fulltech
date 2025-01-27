#pragma once

#include <driver/rmt_rx.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_types.h>
#include <esp_attr.h>
#include <cstdint>


// SOURCE: https://brushlesswhoop.com/dshot-and-bidirectional-dshot/
enum class DShotCommand {
    MOTOR_STOP = 0,           // Currently not implemented(?)
    
    DSHOT_CMD_BEEP1 = 1,                // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP2 = 2,                // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP3 = 3,                // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP4 = 4,                // Wait at least length of beep (260ms) before next command
    DSHOT_CMD_BEEP5 = 5,                // Wait at least length of beep (260ms) before next command
    
    DSHOT_CMD_ESC_INFO = 6,              // Wait at least 12ms before next command
    DSHOT_CMD_SPIN_DIRECTION_1 = 7, //	Need 6x
    DSHOT_CMD_SPIN_DIRECTION_2 = 8, //	Need 6x
    DSHOT_CMD_3D_MODE_OFF = 9, //	Need 6x
    DSHOT_CMD_3D_MODE_ON = 10, //	Need 6x
    DSHOT_CMD_SETTINGS_REQUEST = 11, //	Currently not implemented
    DSHOT_CMD_SAVE_SETTINGS	 = 12, // Need 6x, wait at least 35ms before next command
    DSHOT_EXTENDED_TELEMETRY_ENABLE = 13, //	Need 6x (only on EDT enabed firmware)
    DSHOT_EXTENDED_TELEMETRY_DISABLE = 14, //	Need 6x (only on EDT enabed firmware)

    // 15 - 19, Not assigned yet

    DSHOT_CMD_SPIN_DIRECTION_NORMAL = 20,	//Need 6x
    DSHOT_CMD_SPIN_DIRECTION_REVERSED = 21, // Need 6x

    DSHOT_CMD_LED0_ON = 22,
    DSHOT_CMD_LED1_ON = 23,
    DSHOT_CMD_LED2_ON = 24,
    DSHOT_CMD_LED3_ON = 25,
    DSHOT_CMD_LED0_OFF = 26,
    DSHOT_CMD_LED1_OFF = 27,
    DSHOT_CMD_LED2_OFF = 28,
    DSHOT_CMD_LED3_OFF = 29,


	Audio_Stream_mode = 30, // Currently not implemented
	Silent_mode = 31, //	Currently not implemented
	DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE = 32, //	Need 6x. Disables commands 42 to 47
	DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE = 33, //	Need 6x. Enables commands 42 to 47
	DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY = 34, //	Need 6x. Enables commands 42 to 47 and sends erpm if normal Dshot frame
	DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY = 35, //	Need 6x. Enables commands 42 to 47 and sends erpm period if normal Dshot frame

    // 36 - 41, Not yet assigned

	DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY = 42, //	1°C per LSB
	DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY = 43, //	10mV per LSB, 40.95V max
	DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY = 44, //	100mA per LSB, 409.5A max
	DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY = 45, //	10mAh per LSB, 40.95Ah max
	DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY = 46, //	100erpm per LSB, 409500erpm max
	DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY = 47, //	16us per LSB, 65520us max TBD
                                                        // NOTE: I think lsb here is least significant bit but like in floats(?) since the erpm frame has mantissa and exponent, kerstin pls help
};


class DShotController {
public:
    static constexpr uint32_t DSHOT300_BITRATE = 30000;     // 300kHz
    static constexpr uint32_t DSHOT300_BIT_PERIOD = 3330;   // 3.33us in ns
    static constexpr uint32_t DSHOT300_T0H = 1250;          // 1.25us in ns
    static constexpr uint32_t DSHOT300_T1H = 2500;          // 2.50us in ns

    static constexpr uint32_t FRAME_BITS = 16;              // 16-bit frame
    static constexpr uint32_t THROTTLE_BITS = 1;            // rest is self explanatory
    static constexpr uint32_t TELEMETRY_BITS = 1;
    static constexpr uint32_t CRC_BITS = 4;

                                        // in cpp
    DShotController(gpio_num_t pin);    // this is the constructor
    ~DShotController();                 // this is the destructor

    esp_err_t init();

    esp_err_t arm();

    // See DShotCommand enum for options
    // WARNING: wont block send throttle, so user needs to do it (neccessary for some like DSHOT_CMD_BEEP*)
    esp_err_t send_command(DShotCommand cmd, uint8_t repeat_count = 1);
    
    // sends a throttle value (0.0 - 1.1)
    esp_err_t send_throttle(float throttle, bool telemetry_request = false);

private:
    SemaphoreHandle_t frame_mutex_ = nullptr;
    uint32_t frame_rate_ = 0;
    uint32_t last_frame_time_;

    static uint16_t convert_throttle(float throttle);
    esp_err_t send_throttle_raw(uint16_t throttle, bool telemetry_request = false);
                                                    // in cpp
    uint16_t calculate_crc(uint16_t value) const;   // const at end says that it wont modify the thing given to it (i think?), which makes no sense since its not a pointer(?)
    uint16_t build_frame(uint16_t throttle, bool telemetry_request) const;
    
    static bool IRAM_ATTR tx_done_callback(rmt_channel_handle_t tx_chan, const rmt_tx_done_event_data_t *edata, void *user_data);

    rmt_tx_event_callbacks_t tx_callbacks_ = {};
    
    bool tx_in_progress_ = false;
    bool initialized_ = false;

    gpio_num_t gpio_pin_;
    
    rmt_channel_handle_t tx_channel_ = nullptr;
    rmt_encoder_handle_t encoder_ = nullptr;

    uint16_t current_frame_ = 0;

    static constexpr uint16_t MIN_THROTTLE_VALUE = 48;
    static constexpr uint16_t MAX_THROTTLE_VALUE = 2047;
};
