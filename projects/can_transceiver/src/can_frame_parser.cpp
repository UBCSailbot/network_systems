#include "can_frame_parser.h"

#include <linux/can.h>

#include <cstring>
#include <iostream>
#include <span>
#include <stdexcept>

#include "cmn_hdrs/shared_constants.h"

namespace CAN
{

namespace
{
std::string canDebugStr(const CanId id) { return std::to_string(id) + ": " + CanDescription.at(id); }
}  // namespace

CanIdMismatchException::CanIdMismatchException(std::span<const CanId> valid_ids, const canid_t & received)
{
    std::string build_msg = "Mismatch between received ID: (" + std::to_string(received) + ") and valid IDs: \n";
    for (const CanId & id : valid_ids) {
        build_msg += canDebugStr(id) + "\n";
    }
    msg = build_msg;
}

const char * CanIdMismatchException::what() { return msg.c_str(); }

std::ostream & operator<<(std::ostream & os, const CanBase & can) { return os << can.debugStr(); }

std::string CanBase::debugStr() const { return canDebugStr(id_); }

std::string Battery::debugStr() const
{
    std::stringstream ss;
    ss << CanBase::debugStr() << "\n"
       << "Voltage (V): " << volt_ << "\n"
       << "Current (A): " << curr_ << "\n"
       << "Max voltage (V): " << volt_max_ << "\n"
       << "Min voltage (V): " << volt_min_;
    return ss.str();
}

CanBase::CanBase(uint8_t can_byte_dlen) : can_byte_dlen_(can_byte_dlen) {}

CanBase::CanBase(std::span<const CanId> valid_ids, CanId id, uint8_t can_byte_dlen) : CanBase(can_byte_dlen)
{
    bool valid = std::any_of(valid_ids.begin(), valid_ids.end(), [&id](CanId valid_id) { return id == valid_id; });
    if (!valid) {
        throw CanIdMismatchException(valid_ids, id);
    }
    id_ = id;
}

CanFrame CanBase::toLinuxCan() const { return CanFrame{.can_id = id_, .len = can_byte_dlen_}; }

Battery::Battery(CanId id) : CanBase(std::span{BATTERY_IDS}, id, CAN_BYTE_DLEN_) {}

// NOLINTBEGIN(readability-magic-numbers)
// Way too many bit shifts and masks so have to disable

Battery::Battery(CanFrame cf) : Battery(static_cast<CanId>(cf.can_id))
{
    int16_t raw_volt;
    int16_t raw_curr;
    int16_t raw_max_volt;
    int16_t raw_min_volt;

    std::memcpy(&raw_volt, cf.data, sizeof(int16_t));
    std::memcpy(&raw_curr, cf.data + 2, sizeof(int16_t));
    std::memcpy(&raw_max_volt, cf.data + 4, sizeof(int16_t));
    std::memcpy(&raw_min_volt, cf.data + 6, sizeof(int16_t));

    volt_ = static_cast<float>(raw_volt) / 100;
    curr_ = static_cast<float>(raw_curr) / 100;
    // TODO(hhenry01): Max and min are dodgy... it doesn't make sense to not divide them by 100 - confirm with ELEC
    volt_max_ = static_cast<float>(raw_max_volt);
    volt_min_ = static_cast<float>(raw_min_volt);
}

CanFrame Battery::toLinuxCan() const
{
    int16_t raw_volt = static_cast<int16_t>(volt_ * 100);
    int16_t raw_curr = static_cast<int16_t>(curr_ * 100);
    // TODO(hhenry01): Max and min are dodgy... it doesn't make sense to not multiply them by 100 - confirm with ELEC
    int16_t raw_max_volt = static_cast<int16_t>(volt_max_);
    int16_t raw_min_volt = static_cast<int16_t>(volt_max_);

    CanFrame cf = CanBase::toLinuxCan();
    std::memcpy(cf.data, &raw_volt, sizeof(int16_t));
    std::memcpy(cf.data + 2, &raw_curr, sizeof(int16_t));
    std::memcpy(cf.data + 4, &raw_max_volt, sizeof(int16_t));
    std::memcpy(cf.data + 6, &raw_min_volt, sizeof(int16_t));

    return cf;
}
// NOLINTEND(readability-magic-numbers)

Battery::Battery(custom_interfaces::msg::HelperBattery ros_bat, uint32_t bat_idx) : CanBase(CAN_BYTE_DLEN_)
{
    if (bat_idx >= NUM_BATTERIES) {
        throw std::length_error(
          "bat_idx (" + std::to_string(bat_idx) + ") greater then NUM_BATTERIES (" + std::to_string(NUM_BATTERIES) +
          ")");
    }
    id_       = BATTERY_IDS[bat_idx];
    volt_     = ros_bat.voltage;
    curr_     = ros_bat.current;
    volt_max_ = 0.0;  // UNUSED
    volt_min_ = 0.0;  // UNUSED
}

custom_interfaces::msg::HelperBattery Battery::toRosMsg() const
{
    custom_interfaces::msg::HelperBattery msg;
    msg.set__voltage(volt_);
    msg.set__current(curr_);
    return msg;
}

}  // namespace CAN
