#pragma once

#include <linux/can.h>
#include <stdint.h>

#include <array>
#include <stdexcept>

#include "can_transceiver.h"

enum CanId : uint32_t { Placeholder0 = 0, Placeholder1 = 1, CAN_ID_MAX };
static std::array<std::string, CanId::CAN_ID_MAX> CAN_DEVICE_NAMES = {"Placeholder0", "Placeholder1"};

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

struct CanFrame
{
    union {
        std::array<uint8_t, CAN_MAX_DLEN> raw_data_;
    } data_;
    explicit CanFrame(CanId expected_can_id, const can_frame & frame)
    {
        if (frame.can_id != expected_can_id) {
            throw CanIdMismatchException(expected_can_id, frame.can_id);
        }
        data_.raw_data_ = std::to_array(frame.data);
    }
};

struct Placeholder0 : public CanFrame
{
    static const CanId id_ = CanId::Placeholder0;
    union {
        struct
        {
            uint32_t field_0_       : 31;
            bool     field_0_valid_ : 1;
            uint16_t field_1_       : 16;
            uint8_t  field_2_       : 8;
            uint8_t  field_3_       : 4;
            uint8_t  field_4_       : 4;
        } fields_;
        std::array<uint8_t, CAN_MAX_DLEN> raw_data_;
    } data_;
    static_assert(sizeof(data_) == CAN_MAX_DLEN);

    explicit Placeholder0(const can_frame & frame) : CanFrame(id_, frame) {}
};
