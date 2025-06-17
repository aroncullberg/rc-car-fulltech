//
// Created by aron on 2025-04-26.
//

#pragma once
#include <cstdint>

namespace rc
{
/**
* @brief Enum class representing the channel indices (1..16)
*/
enum class ChannelIndex: std::uint8_t
{
    CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8,
    CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16,
    COUNT
};

inline constexpr std::size_t k_channel_count = static_cast<std::size_t>(ChannelIndex::COUNT);

/**
* @brief Scaled channel value that is expected to be within the range 0 - 2000.
*/
using channel_value_t = int16_t;

struct Channel
{
private:
    channel_value_t value_;

public:
    constexpr explicit Channel(channel_value_t value) noexcept : value_(value)
    {
    }

    constexpr bool low() const noexcept { return value_ < static_cast<channel_value_t>(25); }

    constexpr bool mid() const noexcept
    {
        return value_ >= static_cast<channel_value_t>(1475) && value_ <= static_cast<channel_value_t>(1525);
    }

    constexpr bool high() const noexcept { return value_ > static_cast<channel_value_t>(1975); }

    constexpr channel_value_t raw() const noexcept { return value_; }
    constexpr operator channel_value_t() const noexcept { return value_; }
};
}
