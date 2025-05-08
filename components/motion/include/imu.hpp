//
// Created by aron on 2025-04-30.
//

#pragma once

#include "imu_types.h"

#include <atomic>

namespace proto { class Icm20948Driver; }

namespace motion
{



class Imu {
public:
    static Imu &instance();

    Accel get_accel() const;
    Gyro get_gyro() const;

    constexpr float get_accel_scale() const { return static_cast<float>(2 << accel_fsr_) / 32768.0f; }
    constexpr float get_gyro_scale() const { return static_cast<float>(25 << gyro_fsr_) / 32768.0f; }

    Imu(const Imu &) = delete;
    Imu &operator=(const Imu &) = delete;

private:
    friend class proto::Icm20948Driver;
    Imu() = default;

    void update(const Accel &a);
    void update(const Gyro &g);

    void set_accel_fsr(uint8_t fsr) { accel_fsr_ = fsr; }
    void set_gyro_fsr(uint8_t fsr) { gyro_fsr_ = fsr; }

    uint8_t gyro_fsr_{1};
    uint8_t accel_fsr_{1};

    std::atomic<Accel> accel_{Accel()};
    std::atomic<Gyro> gyro_{Gyro()};
};
}

