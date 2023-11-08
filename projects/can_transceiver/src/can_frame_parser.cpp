#include "can_frame_parser.h"

#include <linux/can.h>

namespace
{
/**
 * @brief Verify that a given CAN frame to construct a device has a valid ID assigned to it
 *
 * @param actual_can_id   ID of the given CAN frame
 * @param expected_can_id ID of the device object that is attempting to be constructed
 */
void checkId(const canid_t & actual_can_id, const CanId & expected_can_id)
{
    if (actual_can_id != expected_can_id) {
        throw CanIdMismatchException(expected_can_id, actual_can_id);
    }
}

/**
 * @brief Default CAN device object constructor for when construction is a 1:1 mapping between raw data and data fields
 *        Copies data from source CAN frame to a given buffer
 *
 * @param frame           Source CAN frame
 * @param expected_can_id Device ID of CAN Frame that is attempting to be constructed
 * @param buf             Output buffer for data to be copied into
 */
void constructCanFrameDefault(
  const can_frame & frame, const CanId & expected_can_id, std::array<uint8_t, CAN_MAX_DLEN> & buf)
{
    if (frame.can_id != expected_can_id) {
        throw CanIdMismatchException(expected_can_id, frame.can_id);
    }
    buf = std::to_array(frame.data);
}
}  // namespace

Placeholder0::Placeholder0(const can_frame & frame) { constructCanFrameDefault(frame, id_, data_.raw_buf_); }

Placeholder1::Placeholder1(const can_frame & frame)
{
    checkId(frame.can_id, id_);
    // NOLINTNEXTLINE(bugprone-narrowing-conversions)
    data_.fields_.field_0_ = static_cast<float>((frame.data[0] + (frame.data[1] << f0_shift_)) / f0_div_);
    data_.fields_.field_1_ = (frame.data[f1_byte_0] + frame.data[f1_byte_1]) & f1_msk_;
}

CanFrame RudderCmd::toLinuxCan()
{
    // PLACEHOLDER - TODO
    CanFrame frame;
    frame.can_id = id_;
    std::copy(std::begin(frame.data), std::end(frame.data), data_.raw_buf_.begin());
    return frame;
}
