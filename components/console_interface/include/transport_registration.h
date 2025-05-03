//
// Created by aron on 5/3/2025.
//

#pragma once

#include "console_transport.h"

extern "C" {
    extern console::ITransport* __start_console_transports[];
    extern console::ITransport* __stop_console_transports[];
}

#define REGISTER_CONSOLE_TRANSPORT(obj) \
    static console::ITransport* __console_transport_ptr_##obj \
        __attribute__((used, section(".console_transports"))) = &obj;

static inline void console_register_static_transports() {
    for (console::ITransport** p = __start_console_transports; p < __stop_console_transports; ++p) {
        console::Engine::instance().register_transport(*p);
    }
}