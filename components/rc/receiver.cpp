//
// Created by cullb on 2025-04-26.
//

#include <receiver.h>

namespace rc
{
    Receiver &Receiver::instance() {
        static Receiver instance;
        return instance;
    }

    ChannelValue Receiver::get(ChannelIndex idx) const {
        return channels_[static_cast<std::size_t>(idx)].load(std::memory_order_relaxed);
    }

    std::array<ChannelValue, kChannelCount> Receiver::getAll() const {
        std::array<ChannelValue, kChannelCount> out{};

        for (size_t i = 0; i < out.size(); ++i) {
            out[i] = channels_[i].load(std::memory_order_relaxed);
        }

        return out;
    }

    void Receiver::update(ChannelIndex idx, ChannelValue value) {
        channels_[static_cast<size_t>(idx)].store(value, std::memory_order_relaxed);
    }





}