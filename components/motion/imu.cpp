//
// Created by aron on 2025-04-30.
//

#include "imu.hpp"

using namespace motion;

Imu &Imu::instance() {
    static Imu instance;
    return instance;
}

Accel Imu::get_accel() const {
    return accel_.load(std::memory_order_relaxed);
}

Gyro Imu::get_gyro() const {
    return gyro_.load(std::memory_order_relaxed);
}

Temp Imu::get_temp() const {
    return temp_.load(std::memory_order_relaxed);
}

void Imu::update(const Accel &a) {
    accel_.store(a, std::memory_order_relaxed);
}

void Imu::update(const Gyro &g) {
    gyro_.store(g, std::memory_order_relaxed);
}

void Imu::update(const Temp &g) {
    temp_.store(g, std::memory_order_relaxed);
}



