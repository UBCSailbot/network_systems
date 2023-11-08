#pragma once

#include <linux/can.h>
#include <stdint.h>

#include <array>
#include <stdexcept>

using CanFrame   = struct can_frame;
using RawDataBuf = std::array<uint8_t, CAN_MAX_DLEN>;

// Enum of all device IDs - TODO: Look into autogenerating this based on a csv file or similar
enum CanId : uint32_t { Placeholder0 = 0, Placeholder1 = 1, RudderCmd, CAN_ID_MAX };
// Array of device names mapped to their device IDs - TODO: Look into autogenerating this based on a csv file or similar
static std::array<std::string, CanId::CAN_ID_MAX> CAN_DEVICE_NAMES = {"Placeholder0", "Placeholder1", "RudderCmd"};

/**
 * Custom exception for when an attempt is made to construct a CAN object with a mismatched ID
 *
 */
class CanIdMismatchException : public std::runtime_error
{
public:
    CanIdMismatchException(const CanId & expected, const canid_t & received)
    : std::runtime_error(
        "Mismatched ID when constructing CAN frame, expected device: " + CAN_DEVICE_NAMES[expected] + " ( " +
        std::to_string(expected) + " ) " +
        " - received device: " + (received >= CanId::CAN_ID_MAX ? "INVALID" : CAN_DEVICE_NAMES[received]) + " ( " +
        std::to_string(received) + " )")
    {
    }
};

/**
 * Placeholder CAN device
 *
 */
struct Placeholder0
{
public:
    // Device id
    static constexpr CanId id_ = CanId::Placeholder0;
    union {
        // Placeholder data fields - elaborate what these fields mean in actual implementation
        struct
        {
            uint32_t field_0_       : 31;
            bool     field_0_valid_ : 1;
            uint16_t field_1_       : 16;
            uint8_t  field_2_       : 8;
            uint8_t  field_3_       : 4;
            uint8_t  field_4_       : 4;
        } fields_;
        // Raw data buffer representation of data
        RawDataBuf raw_buf_;
    } data_;
    static_assert(sizeof(data_) == CAN_MAX_DLEN);

    /**
     * @brief Construct the object from a CAN frame
     *
     * @param frame CAN frame
     */
    explicit Placeholder0(const CanFrame & frame);

    /**
     * @brief Construct the object from a ROS msg
     *
     */
    Placeholder0(/* Placeholder0 ROS msg */);
};

/**
 * Placeholder CAN device
 *
 */
struct Placeholder1
{
public:
    // Device ID
    static constexpr CanId id_ = CanId::Placeholder1;
    union {
        // Placeholder data fields - elaborate what these fields mean in actual implementation
        struct
        {
            float    field_0_;
            uint32_t field_1_;
        } fields_;
        // Raw data buffer representation of data
        RawDataBuf raw_buf_;
    } data_;
    static_assert(sizeof(data_) == CAN_MAX_DLEN);

    /**
     * @brief Construct the object from a CAN frame
     *
     * @param frame CAN frame
     */
    explicit Placeholder1(const CanFrame & frame);

    /**
     * @brief Construct the object from a ROS msg
     *
     */
    Placeholder1(/* Placeholder1 ROS msg */);

private:
    // Constants needed to process data fields
    static constexpr uint8_t  f0_shift_ = 8;
    static constexpr float    f0_div_   = 100.0;
    static constexpr uint8_t  f1_byte_0 = 5;
    static constexpr uint8_t  f1_byte_1 = 6;
    static constexpr uint32_t f1_msk_   = 0xF0F0;
};

/**
 * Rudder Command Frame
 *
 */
struct RudderCmd
{
public:
    // Device ID
    static constexpr CanId id_ = CanId::RudderCmd;
    union {
        // Placeholder data fields - elaborate what these fields mean in actual implementation
        struct
        {
            uint32_t field_0_       : 31;
            bool     field_0_valid_ : 1;
            uint16_t field_1_       : 16;
            uint8_t  field_2_       : 8;
            uint8_t  field_3_       : 4;
            uint8_t  field_4_       : 4;
        } fields_;
        // Raw data buffer representation of data
        RawDataBuf raw_buf_;
    } data_;
    static_assert(sizeof(data_) == CAN_MAX_DLEN);

    /**
     * @brief Convert this object into a standard Linux CAN frame and return it
     *
     * @return Rudder command as a standard Linux CAN frame object
     */
    CanFrame toLinuxCan();
};
