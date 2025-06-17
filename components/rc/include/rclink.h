//
// Created by aron on 2025-04-26.
//

#pragma once

#include <array>
#include <atomic>

#include "esp_err.h"

#include "channel_types.h"


namespace proto
{
class SbusDriver;
};

namespace rc
{
class Backend;
/**
 * @brief Singleton facade that gives application code read-only access to the latest RC channel values,
 * should make it easier to have it be independent of the underlying protocol
 */
class Receiver
{
public:
    static Receiver& instance();

    bool valid_data() const noexcept { return valid_; }

    Channel get(ChannelIndex idx) const;
    std::array<channel_value_t, k_channel_count> get_all() const;

    esp_err_t register_backend(const Backend* b);

    Receiver(const Receiver&) = delete; // Disable copy constructor
    Receiver& operator=(const Receiver&) = delete; // Disable copy assignment operator

private:
    friend class Backend;
    Receiver() = default;

    // void update(ChannelIndex idx, channel_value_t value);
    void push_from_backend(const Backend* caller, ChannelIndex channel, channel_value_t value);
    void set_valid_data(const bool v) { valid_ = v;}

    const Backend* backend_{nullptr};
    bool valid_{false};

    std::array<std::atomic<channel_value_t>, k_channel_count> channels_{};
};
}
