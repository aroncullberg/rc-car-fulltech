//
// Created by aron on 2025-04-26.
//

#pragma once

#include <array>
#include <atomic>
#include <channel_types.h>

#include "channel_types.h"

// NOTE: this is a forward declaration, "alternative" to #include "receiver.h"
namespace proto { class SbusDriver; };

namespace rc
{
    /**
     * @brief Singleton facade that gives application code read-only access to the latest RC channel values,
     * should make it easier to have it be independent of the underlying protocol
     */
    class Receiver {
    public:
        static Receiver &instance();

        ChannelValue get(ChannelIndex idx) const;

        std::array<ChannelValue, kChannelCount> getAll() const;

        Receiver(const Receiver &) = delete;                // Disable copy constructor
        Receiver &operator=(const Receiver &) = delete;     // Disable copy assignment operator

    private:
        Receiver() = default;

        void update(ChannelIndex idx, ChannelValue value);

        friend class proto::SbusDriver;

        std::array<std::atomic<ChannelValue>, kChannelCount> channels_{};
    };
}
