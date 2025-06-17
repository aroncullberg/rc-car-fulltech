//
// Created by cullb on 2025-04-26.
//

#include <esp_log.h>
#include <rclink.h>

#include "channel_types.h"

using namespace rc;

auto tag = "rc-interface";

Receiver& Receiver::instance()
{
    static Receiver instance;
    return instance;
}

Channel Receiver::get(ChannelIndex idx) const
{
    const auto v = channels_[static_cast<size_t>(idx)].load(std::memory_order_relaxed);
    return Channel(v);
}

std::array<channel_value_t, k_channel_count> Receiver::get_all() const
{
    std::array<channel_value_t, k_channel_count> out{};

    for (size_t i = 0; i < out.size(); ++i) {
        out[i] = channels_[i].load(std::memory_order_relaxed);
    }

    return out;
}

esp_err_t Receiver::register_backend(const Backend* b)
{
    if (!b) {
        ESP_LOGE(tag, "register_backend called with nullptr");
        return ESP_ERR_INVALID_ARG;
    }
    if (backend_) {
        ESP_LOGE(tag, "cannot register backend, one is already set");
        return ESP_ERR_INVALID_STATE;
    }
    backend_ = b;
    ESP_LOGI(tag, "registered backend");
    return ESP_OK;
}

void Receiver::push_from_backend(const Backend* caller, ChannelIndex channel, channel_value_t value)
{
    if (caller != backend_) {
        ESP_LOGE("Receiver", "Unregistered backend tried to push data");
        return;
    }
    channels_[static_cast<size_t>(channel)].store(value, std::memory_order_relaxed);
}
