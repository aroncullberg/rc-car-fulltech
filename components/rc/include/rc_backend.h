//
// Created by aron on 06/03/2025.
//

#pragma once

namespace rc
{
class Receiver;

class Backend
{
public:
    virtual ~Backend() = default;
    virtual esp_err_t init() = 0;
    virtual esp_err_t start() = 0;
    virtual esp_err_t stop() = 0;
protected:
    void push(ChannelIndex channel, channel_value_t value) const
    {
        Receiver::instance().push_from_backend(this, channel, value);
    }
};
}
