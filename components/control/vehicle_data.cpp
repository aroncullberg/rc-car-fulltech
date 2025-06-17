//
// Created by aron on 5/9/2025.
//

#include "vehicle_data.hpp"

using namespace  controller;

VehicleData& VehicleData::instance()
{
    static VehicleData instance;
    return instance;
}

bool VehicleData::is_armed() const
{
    return armed.load(std::memory_order_relaxed);
}

void VehicleData::update_armed(bool a)
{
    armed.store(a, std::memory_order_relaxed);
}

std::array<int16_t, 4> VehicleData::get_motor_erpm() const
{
    return erpm.load(std::memory_order_relaxed);
}

void VehicleData::updateErpm(const std::array<int16_t, 4>& new_erpm)
{
    erpm.store(new_erpm, std::memory_order_relaxed);
}


