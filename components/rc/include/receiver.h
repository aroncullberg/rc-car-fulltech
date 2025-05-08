//
// Created by aron on 2025-04-26.
//

#pragma once

#include <array>
#include <atomic>
#include <channel_types.h>


// NOTE: this is a forward declaration, "alternative" to #include "receiver.h"
namespace proto
{
class SbusDriver;
};

namespace rc
{
/**
 * @brief Singleton facade that gives application code read-only access to the latest RC channel values,
 * should make it easier to have it be independent of the underlying protocol
 */
class Receiver
{
public:
    static Receiver& instance();

    using value_type = channel_value_t;


    struct Channel
    {
        using value_type = channel_value_t;

    private:
        value_type value_;

    public:
        constexpr explicit Channel(value_type value) noexcept : value_(value)
        {
        }

        constexpr bool low() const noexcept { return value_ < static_cast<value_type>(500); }

        constexpr bool mid() const noexcept
        {
            return value_ >= static_cast<value_type>(500) && value_ <= static_cast<value_type>(1500);
        }

        constexpr bool high() const noexcept { return value_ > static_cast<value_type>(1500); }

        constexpr value_type raw() const noexcept { return value_; }

        constexpr operator value_type() const noexcept { return value_; }


    };

    Channel get(ChannelIndex idx) const;

    std::array<channel_value_t, k_channel_count> get_all() const;

    Receiver(const Receiver&) = delete; // Disable copy constructor
    Receiver& operator=(const Receiver&) = delete; // Disable copy assignment operator

private:
    Receiver() = default;

    void update(ChannelIndex idx, channel_value_t value);

    friend class proto::SbusDriver;

    std::array<std::atomic<channel_value_t>, k_channel_count> channels_{};
};
}
