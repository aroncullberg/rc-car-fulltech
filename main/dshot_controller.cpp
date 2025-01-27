#include "dshot_controller.hpp"
#include <driver/rmt_encoder.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>

static const char* TAG = "DShotController";

// unsure if i like this cpp fuckery but i leave it like this for now
DShotController::DShotController(gpio_num_t pin) : gpio_pin_(pin) {}

DShotController::~DShotController() {
    if (encoder_) {
        rmt_del_encoder(encoder_);
    }
    if (tx_channel_) {
        rmt_disable(tx_channel_);
        rmt_del_channel(tx_channel_);
    }
}

esp_err_t DShotController::init() {
    if (initialized_) {
        return ESP_OK;
    }
    
    rmt_tx_channel_config_t tx_config = {
        .gpio_num = gpio_pin_,
        .clk_src = RMT_CLK_SRC_DEFAULT, // APB(?) Clock
        .resolution_hz = 40000000, //¯\_(ツ)_/¯
        .mem_block_symbols = 64, // one block sufficent for a(?) dshot frame
        .trans_queue_depth = 1, // single frame quueue because ¯\_(ツ)_/¯
        .flags = {
            .with_dma = false, // only one dma per rx/tx so can not rely on that
            .io_loop_back = false, // 
            .io_od_mode = false     // open train is scary because i dont understand it or what its used for
        }
    };

    esp_err_t err = rmt_new_tx_channel(&tx_config, &tx_channel_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create TX channel");
        return err;
    }    

    tx_callbacks_.on_trans_done = tx_done_callback;
    esp_err_t err = rmt_tx_register_event_callbacks(tx_channel_, &tx_callbacks_, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register TX callbacks");
        return err;
    }

    // ps: encoder does the following, throttle -> RMT symbols
    rmt_bytes_encoder_config_t encoder_config = {};

    rmt_symbol_word_t bit0;
    bit0.duration0 = static_cast<uint16_t>((DSHOT300_T0H * tx_config.resolution_hz) / 1000000000);
    bit0.level0 = 1;
    bit0.duration1 = static_cast<uint16_t>(DSHOT300_BIT_PERIOD * tx_config.resolution_hz / 1000000000 -
                        DSHOT300_T0H * tx_config.resolution_hz / 1000000000);
    bit0.level1 = 0;

    rmt_symbol_word_t bit1;
    bit1.duration0 = static_cast<uint16_t>(DSHOT300_T1H * tx_config.resolution_hz / 1000000000);
    bit1.level0 = 1;
    bit1.duration1 = static_cast<uint16_t>(DSHOT300_BIT_PERIOD * tx_config.resolution_hz / 1000000000 -
                        DSHOT300_T1H * tx_config.resolution_hz / 1000000000);
    bit1.level1 = 0;

    /*
    NOTE: i would do this but that gives me an lsp error, 

    rmt_symbol_word_t bit1 = {
        .duration0 = static_cast<uint16_t>(DSHOT300_T1H * tx_config.resolution_hz / 1000000000),
        .level0 = 1,
        .duration1 = static_cast<uint16_t>(DSHOT300_BIT_PERIOD * tx_config.resolution_hz / 1000000000 -
                        DSHOT300_T1H * tx_config.resolution_hz / 1000000000),
        .level1 = 0,
    };

    and while it compiles, seeing problems: N/A, gives me happy feelings.
    */

    encoder_config.bit0 = bit0;
    encoder_config.bit1 = bit1;
    encoder_config.flags.msb_first = 1; // dshot is most significant bit first: https://dmrlawson.co.uk/index.php/2017/12/04/dshot-in-the-dark/#:~:text=starting%20with%20the%20most%20significant%20bit

    err = rmt_new_bytes_encoder(&encoder_config, &encoder_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create encoder");
        return err;
    }

    err = rmt_enable(tx_channel_);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable channel");
        return err;
    }

    frame_mutex_ = xSemaphoreCreateMutex();
    if (frame_mutex_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    initialized_ = true;
    return ESP_OK;
}

esp_err_t DShotController::arm() {
    // send 0x00 or throttle 48 (0 throttle) for 300ms to arm
    for (int i = 0; i < 30; i++) {  // Send several zero commands to ensure arming
        esp_err_t err = send_throttle(0, false);
        if (err != ESP_OK) {
            return err;
        }
        vTaskDelay(pdMS_TO_TICKS(10));  // Small delay between commands
    }
    return ESP_OK;
}

uint16_t DShotController::calculate_crc(uint16_t value) const {
    return (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
}

uint16_t DShotController::build_frame(uint16_t throttle, bool telemetry_request) const {
    throttle &= 0x07FF; // 11 bit mask
    
    // NOTE: in c++ '|' is a bitwise OR
    // NOTE: (telemetry_request ? 0x01 : 0x00) is just a if this else that
    uint16_t frame = (throttle << 5) | (telemetry_request ? 0x01 : 0x00);

    // NOTE: frame >> 4 works because frame is a 16 bit uint16_t 
    // NOTE: also is |= better? me thinks no since its a weird operator.
    frame = frame | calculate_crc(frame >> 4);

    return frame;
}

esp_err_t DShotController::send_command(DShotCommand cmd, uint8_t repeat_count) {
    uint16_t command_value = static_cast<uint16_t>(cmd);
    if (command_value >= MIN_THROTTLE_VALUE) {
        return ESP_ERR_INVALID_ARG;
    }

    // FIXME: the worst way to do it, its a "temporary" placeholder 
    for (uint8_t i = 0; i < repeat_count; i++) {
        esp_err_t err = send_throttle_raw(command_value, false);
        if (err != ESP_OK) {
            return err;
        }
    }
    return ESP_OK; 
}

uint16_t DShotController::convert_throttle(float throttle) {
    // clamp
    throttle = (throttle < 0.0f) ? 0.0f : (throttle > 1.0f) ? 1.0f : throttle;

    // maps 0.0f - 1.0f -> 48-2047
    return MIN_THROTTLE_VALUE + static_cast<uint16_t>(throttle * (MAX_THROTTLE_VALUE - MIN_THROTTLE_VALUE));
}

esp_err_t DShotController::send_throttle(float throttle, bool telemetry_request) {
    uint16_t dshot_throttle_value = convert_throttle(throttle);
    return send_throttle_raw(dshot_throttle_value, telemetry_request);
}

bool IRAM_ATTR DShotController::tx_done_callback(rmt_channel_handle_t tx_chan, 
    const rmt_tx_done_event_data_t *edata, void *user_data) {

    DShotController* controller = static_cast<DShotController*>(user_data);
    
    // calculate framerate
    uint32_t current_time = esp_timer_get_time();
    controller->frame_rate_ = 1000000 / (current_time - controller->last_frame_time_);
    controller->last_frame_time_ = current_time;

    // simulate bidrectional delay (bad idea maybe?)
    esp_rom_delay_us(30);  // This is the IRAM-safe delay function

    // take mutex without timeout
    if (xSemaphoreTakeFromISR(controller->frame_mutex_, NULL) == pdTRUE) {
        // Then retransmit
        rmt_transmit_config_t tx_config = {
            .loop_count = 0,
            .flags = {
                .eot_level = 0
            }
        };
        rmt_transmit(tx_chan, controller->encoder_, 
                    &controller->current_frame_, sizeof(uint16_t), 
                    &tx_config);

        xSemaphoreGiveFromISR(controller->frame_mutex_, NULL);
    }
    
    return false;
}


esp_err_t DShotController::send_throttle_raw(uint16_t throttle, bool telemetry_request) {
    if (!initialized_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t new_frame = build_frame(throttle, telemetry_request);

    if (xSemaphoreTake(frame_mutex_, pdMS_TO_TICKS(5)) == pdTRUE) {
        current_frame_ = new_frame;

        // if transmission is not running start it
        if (tx_in_progress_) {
            tx_in_progress_ = true;

            rmt_transmit_config_t tx_config = {
                .loop_count = 0, // no loop de loop
                .flags = {
                    .eot_level = 0 // end with low level
                }
            };

            esp_err_t err = rmt_transmit(tx_channel_, encoder_, &current_frame_, sizeof(current_frame_), &tx_config);
            xSemaphoreGive(frame_mutex_);
            return err;
        }

        xSemaphoreGive(frame_mutex_);
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}


