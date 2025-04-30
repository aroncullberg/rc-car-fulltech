//
// Created by aron on 2025-04-30.
//

#include "imu.h"

using namespace motion;

Imu &Imu::instance() {
    static Imu instance;
    return instance;
}

Accel Imu::getAccel() const {
    return accel_.load(std::memory_order_relaxed);
}

Gyro Imu::getGyro() const {
    return gyro_.load(std::memory_order_relaxed);
}

Quat6 Imu::getQuat6() const {
    return quat6_.load(std::memory_order_relaxed);
}

Quat9 Imu::getQuat9() const {
    return quat9_.load(std::memory_order_relaxed);
}

Stats Imu::getStats() const {
    return stats_.load(std::memory_order_relaxed);
}

void Imu::update(const Accel &a) {
    accel_.store(a, std::memory_order_relaxed);
}

void Imu::update(const Gyro &g) {
    gyro_.store(g, std::memory_order_relaxed);
}

void Imu::update(const Quat6 &q6) {
    quat6_.store(q6, std::memory_order_relaxed);
}

void Imu::update(const Quat9 &q9) {
    quat9_.store(q9, std::memory_order_relaxed);
}

void Imu::update(const Stats &s) {
    stats_.store(s, std::memory_order_relaxed);
}



