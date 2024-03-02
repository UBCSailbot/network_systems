#include "can_frame_parser.h"

#include <linux/can.h>

#include <cstring>
#include <iostream>
#include <span>
#include <stdexcept>

#include "cmn_hdrs/shared_constants.h"
#include "utils/utils.h"

namespace CAN_FP
{

namespace
{
/**
 * @brief Convert a CanId to its string representation
 *
 * @param id CanId to convert
 * @return std::string
 */
std::string CanIdToStr(CanId id) { return std::to_string(static_cast<canid_t>(id)); }

/**
 * @brief Get a debug string for a CanId
 *
 * @param id CanId to target
 * @return std::string
 */
std::string CanDebugStr(CanId id) { return CanIdToStr(id) + ": " + CAN_DESC.at(id); }
}  // namespace

// CanIdMismatchException START

CanIdMismatchException::CanIdMismatchException(std::span<const CanId> valid_ids, CanId received)
{
    std::string build_msg = "Mismatch between received ID: (" + CanIdToStr(received) + ") and valid IDs: \n";
    for (const CanId & id : valid_ids) {
        build_msg += CanDebugStr(id) + "\n";
    }
    msg_ = build_msg;
}

const char * CanIdMismatchException::what() { return msg_.c_str(); }

// CanIdMismatchException END
// BaseFrame START
// BaseFrame public START

std::ostream & operator<<(std::ostream & os, const BaseFrame & can) { return os << can.debugStr(); }

// BaseFrame public END
// BaseFrame protected START

BaseFrame::BaseFrame(CanId id, uint8_t can_byte_dlen) : id_(id), can_byte_dlen_(can_byte_dlen) {}

BaseFrame::BaseFrame(std::span<const CanId> valid_ids, CanId id, uint8_t can_byte_dlen) : BaseFrame(id, can_byte_dlen)
{
    bool valid = std::any_of(valid_ids.begin(), valid_ids.end(), [&id](CanId valid_id) { return id == valid_id; });
    if (!valid) {
        throw CanIdMismatchException(valid_ids, id);
    }
}

std::string BaseFrame::debugStr() const { return CanDebugStr(id_); }

CanFrame BaseFrame::toLinuxCan() const { return CanFrame{.can_id = static_cast<canid_t>(id_), .len = can_byte_dlen_}; }

// BaseFrame protected END
// BaseFrame END
// Battery START
// Battery public START

Battery::Battery(const CanFrame & cf) : Battery(static_cast<CanId>(cf.can_id))
{
    int16_t raw_volt;
    int16_t raw_curr;
    int16_t raw_max_volt;
    int16_t raw_min_volt;

    std::memcpy(&raw_volt, cf.data + BYTE_OFF_VOLT, sizeof(int16_t));
    std::memcpy(&raw_curr, cf.data + BYTE_OFF_CURR, sizeof(int16_t));
    std::memcpy(&raw_max_volt, cf.data + BYTE_OFF_MAX_VOLT, sizeof(int16_t));
    std::memcpy(&raw_min_volt, cf.data + BYTE_OFF_MIN_VOLT, sizeof(int16_t));

    volt_ = static_cast<float>(raw_volt) / 100;  // NOLINT(readability-magic-numbers)
    curr_ = static_cast<float>(raw_curr) / 100;  // NOLINT(readability-magic-numbers)

    // TODO(hhenry01): Max and min are dodgy... it doesn't make sense to not divide them by 100 - confirm with ELEC
    volt_max_ = static_cast<float>(raw_max_volt);
    volt_min_ = static_cast<float>(raw_min_volt);

    checkBounds();
}

Battery::Battery(msg::HelperBattery ros_bat, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), volt_(ros_bat.voltage), curr_(ros_bat.current), volt_max_(0.0), volt_min_(0.0)
{
    checkBounds();
}

msg::HelperBattery Battery::toRosMsg() const
{
    msg::HelperBattery msg;
    msg.set__voltage(volt_);
    msg.set__current(curr_);

    std::vector<msg::HelperBattery> batteries;
    batteries.push_back(msg);

    if (batteries.size() >= 2) {
        std::array<msg::HelperBattery, 2> msg_arr;
        copy(batteries.begin(), batteries.end(), msg_arr.begin());
    }

    return msg;
}

CanFrame Battery::toLinuxCan() const
{
    int16_t raw_volt = static_cast<int16_t>(volt_ * 100);  // NOLINT(readability-magic-numbers)
    int16_t raw_curr = static_cast<int16_t>(curr_ * 100);  // NOLINT(readability-magic-numbers)
    // TODO(hhenry01): Max and min are dodgy... it doesn't make sense to not multiply them by 100 - confirm with ELEC
    int16_t raw_max_volt = static_cast<int16_t>(volt_max_);
    int16_t raw_min_volt = static_cast<int16_t>(volt_max_);

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_VOLT, &raw_volt, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_CURR, &raw_curr, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_MAX_VOLT, &raw_max_volt, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_MIN_VOLT, &raw_min_volt, sizeof(int16_t));

    return cf;
}

std::string Battery::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Voltage (V): " << volt_ << "\n"
       << "Current (A): " << curr_ << "\n"
       << "Max voltage (V): " << volt_max_ << "\n"
       << "Min voltage (V): " << volt_min_;
    return ss.str();
}

std::optional<CanId> Battery::rosIdxToCanId(size_t bat_idx)
{
    if (bat_idx < BATTERY_IDS.size()) {
        return BATTERY_IDS[bat_idx];
    }
    return std::nullopt;
}

// Battery public END
// Battery private START

Battery::Battery(CanId id) : BaseFrame(std::span{BATTERY_IDS}, id, CAN_BYTE_DLEN_) {}

void Battery::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(volt_, BATT_VOLT_LBND, BATT_VOLT_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Battery voltage is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(curr_, BATT_CURR_LBND, BATT_CURR_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Battery current is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// Battery private END
// Battery END

// SailCmd START
// SailCmd public START

SailCmd::SailCmd(const CanFrame & cf) : SailCmd(static_cast<CanId>(cf.can_id))
{
    int16_t raw_angle;

    std::memcpy(&raw_angle, cf.data + BYTE_OFF_ANGLE, sizeof(int16_t));

    angle_ = static_cast<float>(raw_angle);

    checkBounds();
}

SailCmd::SailCmd(msg::SailCmd ros_sail_cmd, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), angle_(ros_sail_cmd.trim_tab_angle_degrees)
{
    checkBounds();
}

msg::SailCmd SailCmd::toRosMsg() const
{
    msg::SailCmd msg;
    msg.set__trim_tab_angle_degrees(angle_);
    return msg;
}

CanFrame SailCmd::toLinuxCan() const
{
    int16_t raw_angle = static_cast<int16_t>(angle_);

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_ANGLE, &raw_angle, sizeof(int16_t));

    return cf;
}

std::string SailCmd::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Trim tab angle (degrees): " << angle_;
    return ss.str();
}

// SailCmd public END
// SailCmd private START

SailCmd::SailCmd(CanId id) : BaseFrame(std::span{SAIL_CMD_IDS}, id, CAN_BYTE_DLEN_) {}

void SailCmd::checkBounds() const
{  //fix min max angle
    auto err = utils::isOutOfBounds<float>(angle_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Sail angle is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// SailCmd private END
// SailCmd END

// WindSensor START
// WindSensor public START

WindSensor::WindSensor(const CanFrame & cf) : WindSensor(static_cast<CanId>(cf.can_id))
{
    int16_t raw_wind_speed;
    int16_t raw_wind_dir;

    std::memcpy(&raw_wind_speed, cf.data + BYTE_OFF_SPEED, sizeof(int16_t));
    std::memcpy(&raw_wind_dir, cf.data + BYTE_OFF_ANGLE, sizeof(int16_t));

    wind_speed_ = static_cast<float>(raw_wind_speed) * 1.852 / 10.0;  //NOLINT
    wind_angle_ = static_cast<int16_t>(raw_wind_dir);

    checkBounds();
}

WindSensor::WindSensor(msg::WindSensor ros_wind_sensor, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_), wind_angle_(ros_wind_sensor.direction), wind_speed_(ros_wind_sensor.speed.speed)
{
    checkBounds();
}

msg::WindSensor WindSensor::toRosMsg() const
{
    msg::WindSensor  msg;
    msg::HelperSpeed speed;
    msg.set__direction(static_cast<int16_t>(wind_angle_));
    speed.set__speed(wind_speed_);
    msg.set__speed(speed);
    return msg;
}

CanFrame WindSensor::toLinuxCan() const
{
    int16_t raw_wind_speed = static_cast<int16_t>(wind_speed_ * 10 / 1.852);  //NOLINT
    int16_t raw_wind_dir   = static_cast<int16_t>(wind_angle_);

    CanFrame cf = BaseFrame::toLinuxCan();
    std::memcpy(cf.data + BYTE_OFF_SPEED, &raw_wind_speed, sizeof(int16_t));
    std::memcpy(cf.data + BYTE_OFF_ANGLE, &raw_wind_dir, sizeof(int16_t));

    return cf;
}

std::string WindSensor::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Wind speed (m/s): " << wind_speed_ << "\n"
       << "Wind angle (degrees): " << wind_angle_;
    return ss.str();
}

std::optional<CanId> WindSensor::rosIdxToCanId(size_t wind_idx)
{
    if (wind_idx < WIND_SENSOR_IDS.size()) {
        return WIND_SENSOR_IDS[wind_idx];
    }
    return std::nullopt;
}
// WindSensor public END
// WindSensor private START

WindSensor::WindSensor(CanId id) : BaseFrame(std::span{WIND_SENSOR_IDS}, id, CAN_BYTE_DLEN_) {}

void WindSensor::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(wind_angle_, WIND_DIRECTION_LBND, WIND_DIRECTION_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Wind angle is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(wind_speed_, SPEED_LBND, SPEED_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Wind speed is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

// WindSensor private END
// WindSensor END

// GPS START
// GPS public START

GPS::GPS(const CanFrame & cf) : GPS(static_cast<CanId>(cf.can_id)) { updateRos(cf); }

GPS::GPS(msg::GPS ros_gps, CanId id)
: BaseFrame(id, CAN_BYTE_DLEN_),
  lon_(ros_gps.lat_lon.longitude),
  lat_(ros_gps.lat_lon.latitude),
  speed_(ros_gps.speed.speed),
  heading_(ros_gps.heading.heading)
{
    checkBounds();
}

msg::GPS GPS::toRosMsg() const
{
    msg::GPS           msg;
    msg::HelperLatLon  lat_lon;
    msg::HelperSpeed   speed;
    msg::HelperHeading heading;

    lat_lon.set__latitude(lat_);
    lat_lon.set__longitude(lon_);
    speed.set__speed(speed_);
    heading.set__heading(heading_);

    msg.set__lat_lon(lat_lon);
    msg.set__speed(speed);
    msg.set__heading(heading);
    return msg;
}

CanFrame GPS::toLinuxCan(int frame_index) const
{
    CanFrame cf = BaseFrame::toLinuxCan();
    int8_t   raw_degree;
    float    raw_minute;

    int16_t raw_speed;
    int16_t raw_track;
    int16_t raw_mag_var;
    int16_t raw_heading;

    switch (frame_index) {
        case 0:
            raw_degree = static_cast<int8_t>(lat_);
            raw_minute = (lat_ - raw_degree) * 60.0;  // NOLINT
            std::memcpy(cf.data + BYTE_OFF_DEG, &raw_degree, sizeof(int8_t));
            std::memcpy(cf.data + BYTE_OFF_MIN, &raw_minute, sizeof(int32_t));
            break;
        case 1:
            raw_degree = static_cast<int8_t>(lon_);
            raw_minute = (lat_ - raw_degree) * 60.0;  // NOLINT
            std::memcpy(cf.data + BYTE_OFF_DEG, &raw_degree, sizeof(int8_t));
            std::memcpy(cf.data + BYTE_OFF_MIN, &raw_minute, sizeof(int32_t));
            break;
        case 2:
            raw_speed   = static_cast<int16_t>(speed_);
            raw_mag_var = static_cast<int16_t>(mag_var_);
            raw_heading = static_cast<int16_t>(heading_);
            raw_track   = static_cast<int16_t>(track_);
            std::memcpy(cf.data + BYTE_OFF_SPEED, &raw_speed, sizeof(int16_t));
            std::memcpy(cf.data + BYTE_OFF_HEADING, &raw_heading, sizeof(int16_t));
            std::memcpy(cf.data + BYTE_OFF_MAG_VAR, &raw_mag_var, sizeof(int16_t));
            std::memcpy(cf.data + BYTE_OFF_TRACK, &raw_track, sizeof(int16_t));
            break;
        default:
            throw std::out_of_range("Invalid frame index for GPS!");
    }
    return cf;
}

void GPS::updateRos(CanFrame cf)
{
    int8_t  raw_degree;
    float   raw_minute;
    int16_t raw_speed;
    int16_t raw_heading;
    int16_t raw_track;
    int16_t raw_mag_var;

    //note: GPS_IDS[0] is latitude, GPS_IDS[1] is longitude, GPS_IDS[2] is speed, heading, track, and magnetic variation
    switch (cf.can_id) {
        case static_cast<canid_t>(GPS_IDS[0]):
            std::memcpy(&raw_degree, cf.data + BYTE_OFF_DEG, sizeof(int8_t));
            std::memcpy(&raw_minute, cf.data + BYTE_OFF_MIN, sizeof(int32_t));
            lat_ = static_cast<float>(raw_degree + static_cast<float>(raw_minute) / 60.0);  //NOLINT
            break;

        case static_cast<canid_t>(GPS_IDS[1]):
            std::memcpy(&raw_degree, cf.data + BYTE_OFF_DEG, sizeof(int8_t));
            std::memcpy(&raw_minute, cf.data + BYTE_OFF_MIN, sizeof(int32_t));
            lon_ = static_cast<float>(raw_degree + static_cast<float>(raw_minute) / 60.0);  //NOLINT
            break;

        case static_cast<canid_t>(GPS_IDS[2]):
            std::memcpy(&raw_speed, cf.data + BYTE_OFF_SPEED, sizeof(int16_t));
            std::memcpy(&raw_heading, cf.data + BYTE_OFF_HEADING, sizeof(int16_t));
            std::memcpy(&raw_mag_var, cf.data + BYTE_OFF_MAG_VAR, sizeof(int16_t));
            std::memcpy(&raw_track, cf.data + BYTE_OFF_TRACK, sizeof(int16_t));
            speed_   = static_cast<float>(raw_speed);
            heading_ = static_cast<float>(raw_heading);
            track_   = static_cast<float>(raw_track);
            mag_var_ = static_cast<float>(raw_mag_var);
            break;

        default:
            throw CanIdMismatchException(GPS_IDS, static_cast<CanId>(cf.can_id));
    }

    checkBounds();
}

std::string GPS::debugStr() const
{
    std::stringstream ss;
    ss << BaseFrame::debugStr() << "\n"
       << "Latitude: " << lat_ << "\n"
       << "Longitude: " << lon_ << "\n"
       << "Speed (m/s): " << speed_ << "\n"
       << "Heading (degrees): " << heading_;
    return ss.str();
}

// GPS public END
// GPS private START

GPS::GPS(CanId id) : BaseFrame(std::span{GPS_IDS}, id, CAN_BYTE_DLEN_) {}

void GPS::checkBounds() const
{
    auto err = utils::isOutOfBounds<float>(lat_, LAT_LBND, LAT_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Latitude is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(lon_, LON_LBND, LON_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Longitude is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(speed_, SPEED_LBND, SPEED_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Speed is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
    err = utils::isOutOfBounds<float>(heading_, HEADING_LBND, HEADING_UBND);
    if (err) {
        std::string err_msg = err.value();
        throw std::out_of_range("Heading is out of bounds!\n" + debugStr() + "\n" + err_msg);
    }
}

}  // namespace CAN_FP
