#include "can_frame_parser.h"

#include <linux/can.h>

#include <cstring>
#include <span>

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

// std::ostream & operator<<(std::ostream & os, const Battery & can)
// {
//     return os << static_cast<CanBase>(can) << std::endl
//               << "Voltage (V): " << can.volt_ << std::endl
//               << "Current (A): " << can.curr_ << std::endl
//               << "Max voltage (V): " << can.volt_max_ << std::endl
//               << "Min voltage (V): " << can.volt_min_;
// }

CanBase::CanBase(std::span<const CanId> valid_ids, CanId id)
{
    bool valid = std::any_of(valid_ids.begin(), valid_ids.end(), [&id](CanId valid_id) { return id == valid_id; });
    if (!valid) {
        throw CanIdMismatchException(valid_ids, id);
    }
    id_ = id;
}

// NOLINTBEGIN(readability-magic-numbers)
// Way too many bit shifts and masks so have to disable

Battery::Battery(CanFrame cf) : CanBase(std::span{BATTERY_IDS}, static_cast<CanId>(cf.can_id))
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

// NOLINTEND(readability-magic-numbers)

}  // namespace CAN
