//
// Created by cullb on 2025-04-26.
//

#include "elrs_backend.h"
#include "ESP_CRSF.h"

using rc::channel_value_t;
using rc::ChannelIndex;

#define TAG "ExpressLRS"

namespace proto
{
ExpressLRS::ExpressLRS(const Config& cfg) : cfg_(cfg)
{
};

ExpressLRS::~ExpressLRS()
{
    ExpressLRS::stop();
    ESP_LOGI(TAG, "ExpressLRS instance destroyed");
}

// TODO: change configureUART to return ESP_ERR_... if something goes wrong
esp_err_t ExpressLRS::init()
{
    crsf_config_t config = {
        .uart_num = static_cast<uint8_t>(cfg_.uart_num),
        .tx_pin = static_cast<uint8_t>(cfg_.uart_tx_pin),
        .rx_pin = static_cast<uint8_t>(cfg_.uart_rx_pin)
    };
    CRSF_init(&config);
    return ESP_OK;
}

esp_err_t ExpressLRS::start()
{
    if (running_)
        return ESP_ERR_INVALID_STATE;
    BaseType_t res = xTaskCreatePinnedToCore(
        taskEntry,
        "ExpressLRS task",
        4096,
        this,
        5,
        &task_,
        1
    );

    running_ = (res == pdPASS);

    return running_ ? ESP_OK : ESP_FAIL;
}

esp_err_t ExpressLRS::stop()
{
    if (!running_)
        return ESP_ERR_INVALID_STATE;
    vTaskDelete(task_);
    task_ = nullptr;
    running_ = false;
    return ESP_OK;
}


void ExpressLRS::taskEntry(void* arg)
{
    auto* self = static_cast<ExpressLRS*>(arg);
    self->run();
}

void ExpressLRS::run()
{
    crsf_channels_t channels = {0};

    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        CRSF_receive_channels(&channels);

        channel_value_t scaled={};
        scaled = rawToScaled(channels.ch1);
        push(ChannelIndex::CH1, scaled);
        scaled = rawToScaled(channels.ch2);
        push(ChannelIndex::CH2, scaled);
        scaled = rawToScaled(channels.ch3);
        push(ChannelIndex::CH3, scaled);
        scaled = rawToScaled(channels.ch4);
        push(ChannelIndex::CH4, scaled);
        scaled = rawToScaled(channels.ch5);
        push(ChannelIndex::CH5, scaled);
        scaled = rawToScaled(channels.ch6);
        push(ChannelIndex::CH6, scaled);
        scaled = rawToScaled(channels.ch7);
        push(ChannelIndex::CH7, scaled);
        scaled = rawToScaled(channels.ch8);
        push(ChannelIndex::CH8, scaled);
        scaled = rawToScaled(channels.ch9);
        push(ChannelIndex::CH9, scaled);
        scaled = rawToScaled(channels.ch10);
        push(ChannelIndex::CH10, scaled);
        scaled = rawToScaled(channels.ch11);
        push(ChannelIndex::CH11, scaled);
        scaled = rawToScaled(channels.ch12);
        push(ChannelIndex::CH12, scaled);
        scaled = rawToScaled(channels.ch13);
        push(ChannelIndex::CH13, scaled);
        scaled = rawToScaled(channels.ch14);
        push(ChannelIndex::CH14, scaled);
        scaled = rawToScaled(channels.ch15);
        push(ChannelIndex::CH15, scaled);
        scaled = rawToScaled(channels.ch16);
        push(ChannelIndex::CH16, scaled);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(10));
    }
}


}
