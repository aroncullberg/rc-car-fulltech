#pragma once

#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "config_manager.h"

class Servo {
public:
    struct Config {
        gpio_num_t gpio_num{GPIO_NUM_NC};  // GPIO pin number
        int min_pulse_width_us{500};       // Minimum pulse width in microseconds
        int max_pulse_width_us{2500};      // Maximum pulse width in microseconds
        uint32_t freq_hz{50};              // PWM frequency in Hz (standard servo is 50Hz)
    };

    void updateFromConfig();
    ConfigManager::ConfigChangeCallback callback_;

    explicit Servo(const Config& config);
    ~Servo();

    // Delete copy operations
    Servo(const Servo&) = delete;
    Servo& operator=(const Servo&) = delete;

    // Set position from -1.0 to 1.0
    esp_err_t setPosition(uint16_t position);

private:
    esp_err_t init();
    uint32_t calculateCompareValue(uint16_t position) ;

    Config config_;
    mcpwm_timer_handle_t timer_{nullptr};
    mcpwm_oper_handle_t operator_{nullptr};
    mcpwm_cmpr_handle_t comparator_{nullptr};
    mcpwm_gen_handle_t generator_{nullptr};

    int min_pulse_width_us_{500};  // Minimum pulse width in microseconds
    int max_pulse_width_us_{2500};  // Maximum pulse width in microseconds
    int center_pulse_width_us_{1500};  // Maximum pulse width in microseconds
    int offset_{0};
    bool invert_steering_{true};

    static constexpr uint32_t TIMEBASE_RESOLUTION_HZ = 1000000;  // 1MHz, 1us per tick
    static constexpr const char* TAG = "Servo";
};
