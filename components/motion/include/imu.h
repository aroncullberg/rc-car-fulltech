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

    Accel getAccel() const;
    Gyro getGyro() const;
    Quat6 getQuat6() const;
    Quat9 getQuat9() const;
    Stats getStats() const;

    Imu(const Imu &) = delete;
    Imu &operator=(const Imu &) = delete;

private:
    friend class proto::Icm20948Driver;
    Imu() = default;

    void update(const Accel &a);
    void update(const Gyro &g);
    void update(const Quat6 &q6);
    void update(const Quat9 &q9);
    void update(const Stats &s);

    std::atomic<Accel> accel_{Accel()};
    std::atomic<Gyro> gyro_{Gyro()};
    std::atomic<Quat6> quat6_{Quat6()};
    std::atomic<Quat9> quat9_{Quat9()};
    std::atomic<Stats> stats_{Stats()};
};
}

