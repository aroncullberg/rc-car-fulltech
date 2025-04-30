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
    enum class ChannelIndex: std::size_t {
        CH1 = 0, CH2, CH3, CH4, CH5, CH6, CH7, CH8,
        CH9, CH10, CH11, CH12, CH13, CH14, CH15, CH16,
        COUNT
    };

    inline constexpr std::size_t kChannelCount = static_cast<std::size_t>(ChannelIndex::COUNT);

    /**
    * @brief Scaled channel value that is expected to be within the range 0 - 2000.
    */
    using ChannelValue = int16_t;
}
