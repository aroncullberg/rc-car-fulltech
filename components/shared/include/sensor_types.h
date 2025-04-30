#pragma once

#include <array>

namespace sensor
{
    // struct SbusQuality {
    //     uint8_t frame_loss_percent{0};
    //     uint32_t error_count{0};
    //     uint16_t frame_interval_ms{0};
    //     bool valid_signal{false};
    // };
    //
    // struct SbusData {
    //     std::array<uint16_t, 16> channels_raw{};                // 11 bit values
    //     std::array<rc::ChannelValue, 16> channels_scaled{};     // 0-2000 scaled values
    //     SbusQuality quality{};
    // };

    //
    // constexpr uint16_t RAW_MIN = 192;
    // constexpr uint16_t RAW_MAX = 1792;
    // constexpr uint16_t RAW_RANGE = RAW_MAX - RAW_MIN;
    // constexpr channel_t SCALED_MIN = 0;
    // constexpr channel_t SCALED_MAX = 2000;

    // constexpr std::array<channel_t, RAW_RANGE + 1> createScaleLookupTable() {
    //     std::array<int16_t, RAW_RANGE + 1> table{};
    //
    //     for (int16_t i = 0; i <= RAW_RANGE; ++i) {
    //         // (i * SCALED_MAX + RAW_RANGE/2) / RAW_RANGE
    //         table[i] = static_cast<uint16_t>((static_cast<uint32_t>(i) * SCALED_MAX + (RAW_RANGE / 2)) / RAW_RANGE);
    //     }
    //
    //     return table;
    // }
    //
    // constexpr auto SBUS_SCALE_TABLE = createScaleLookupTable();

    // constexpr channel_t rawToScaled(const uint16_t raw_value) {
    //     const uint16_t index = std::clamp(raw_value, RAW_MIN, RAW_MAX) - RAW_MIN;
    //     return SBUS_SCALE_TABLE[index];
    // }

    // struct SbusData {
    //     uint16_t channels_raw[16]{1000};
    //     // TODO: Make this use a custon type that is just a wrapper for uint16 so i can force functions to only accept channel value type.
    //     channel_t channels_scaled[16]{1000};
    //
    //     // std::map<uint16_t, SbusChannel> channels;
    //     struct {
    //         uint8_t frame_loss_percent{0};
    //         uint32_t error_count{0};
    //         float frame_interval_ms{0.0f};
    //         bool valid_signal{false};
    //     } quality;
    //
    //     SbusData() = default;
    // };

    struct ImuData {
        // Using GMP_4 which means the values are ... )TODO=
        int16_t accel_x{0}; // 1 unit = 1/80?? g // (positive) X is forwards movement
        int16_t accel_y{0}; // 1 unit = 1/80?? g // (positive) Y is right movement
        int16_t accel_z{0};
        // 1 unit = 1/80?? g // (positive) Z is downwards movement (down because of NED coordinate system)


        static constexpr float GYRO_TO_DPS = 500.0f / 32768.0f;
        // right hand rule, thumb points in direction of positive axis fingers in direction of rotation
        int16_t gyro_x{0}; // (positive) X is roll right
        int16_t gyro_y{0}; // (positive) Y is front side up rear down, pitch up
        int16_t gyro_z{0}; // (positive) Z is yaw right (clockwise when looking from above)

        // Quaternion orientation (Q30 format)
        int32_t quat9_x{0}; // i component // (positive) X is roll right
        int32_t quat9_y{0}; // j component // (positive) Y is pitch up
        int32_t quat9_z{0}; // k component // (positive) Z is yaw right (clockwise when looking from above)
        uint16_t quat9_accuracy{0}; // DMP accuracy indicator

        // game roation vector (for smooth rotations)
        int32_t quat6_x{0};
        int32_t quat6_y{0};
        int32_t quat6_z{0};

        // Quality metrics
        struct {
            bool valid_data{false}; // Indicates if data is valid
            uint32_t error_count{0}; // Cumulative error counter
            float update_rate_hz{0.0f}; // Current update rate
            float accel_accuracy{0.0f};
            float gyro_accuracy{0.0f};
        } quality;

        struct Bias {
            struct {
                int16_t x{0};
                int16_t y{0};
                int16_t z{0};
            } gyro;
            struct {
                int16_t x{0};
                int16_t y{0};
                int16_t z{0};
            } accel;
            struct {
                int16_t x{0};
                int16_t y{0};
                int16_t z{0};
            } mag;

            Bias() = default;
        } bias;

        // Default constructor for zero-initialization
        ImuData() = default;
    };

    struct Servo {
        static constexpr uint32_t US_FAILSAFE = 1500;
        static constexpr int16_t FAILSAFE_POSITION = 1000;
        static constexpr int16_t MIN_POSITION = 0;
        static constexpr int16_t MAX_POSITION = 2000;
        static constexpr int16_t NEUTRAL_POSITION = 1000;
    };

    struct Motor {
        static constexpr int16_t FAILSAFE_THROTTLE = 0;
    };


    struct eRPMData // TODO: This should be a union with the throttle value
    {
        struct FrontRight {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } front_right;

        struct FrontLeft {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } front_left;

        struct RearLeft {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } rear_left;

        struct RearRight {
            uint32_t erpm{0};
            uint16_t error_rate{0}; // TODO: need to decide on scale
        } rear_right;

        int16_t throttle_value{0};
    };
} // Namespace sensor
