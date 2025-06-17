//
// Created by aron on 5/9/2025.
//

#pragma once

#include <array>
#include <atomic>

namespace controller
{
class VehicleController;

/**
 *
 */
class VehicleData
{
public:
    static VehicleData& instance();

    bool is_armed() const;
    std::array<int16_t, 4> get_motor_erpm() const;

    VehicleData(const VehicleData&) = delete; // Disable copy constructor
    VehicleData& operator=(const VehicleData&) = delete; // Disable copy assignment operator
private:
    friend class controller::VehicleController;
    VehicleData() = default;

    void update_armed(bool armed);
    void updateErpm(const std::array<int16_t, 4>& new_erpm);

    std::atomic<std::array<int16_t, 4>> erpm{};
    std::atomic<bool> armed{};
};

}



