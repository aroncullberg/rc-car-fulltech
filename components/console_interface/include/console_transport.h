//
// Created by aron on 5/3/2025.
//

#pragma once

#include <cstddef>

#include "esp_err.h"

namespace console
{
class Engine;

class ITransport
{
public:
    virtual ~ITransport() = default;

    virtual esp_err_t start() = 0;

    virtual void write(const char* data, size_t len) = 0;

protected:
    void on_line(const char* line);
};
}
