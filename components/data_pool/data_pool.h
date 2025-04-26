#pragma once

#include <atomic>

// #include "gps.h"
#include "imu.h"
#include "sensor_types.h"

class VehicleData {
    public:
    // [SIGNLETON] Access method (global?)
    static VehicleData& instance() {
        static VehicleData instance; // Should be thread safe (?)
        return instance;
    }

    // thread safe (i think since only one task will call it?) update methods
    void updateGPS(const sensor::GpsData& data);
    void updateIMU(const sensor::ImuData& data);
    void updateErpm(const sensor::eRPMData& data);

    // Unsure if this is threadsafe, should be fine since its read and not write(?)
    sensor::GpsData getGPS() const;
    sensor::ImuData getImu() const;
    sensor::eRPMData getErpm() const;

    uint32_t getGPSTimestamp() const {return gps_timestamp_.load() ; }
    uint32_t getImuTimestamp() const {return imu_timestamp_.load() ; }

    
    private:
    // [SINGLETON] private constructor to prevent direct instantiation
    VehicleData() = default;

    // [SINGLETON] Delete compy constructor and assignment operator
    VehicleData(const VehicleData&) = delete;
    VehicleData& operator =(const VehicleData&) = delete;

    sensor::GpsData gps_{};
    sensor::ImuData imu_{};
    sensor::eRPMData eRPM_{};

    std::atomic<uint32_t> gps_timestamp_{0};
    std::atomic<uint32_t> imu_timestamp_{0};
};