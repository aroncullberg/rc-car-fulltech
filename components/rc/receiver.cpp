//
// Created by cullb on 2025-04-26.
//

#include <receiver.h>

#include "channel_types.h"

using namespace rc;

Receiver& Receiver::instance()
{
    static Receiver instance;
    return instance;
}

Receiver::Channel Receiver::get(ChannelIndex idx) const
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

void Receiver::update(ChannelIndex idx, channel_value_t value)
{
    channels_[static_cast<size_t>(idx)].store(value, std::memory_order_relaxed);
}
