#pragma once

#include <linux/can.h>
#include <stdint.h>

#include <array>
#include <custom_interfaces/msg/batteries.hpp>
#include <map>
#include <span>
#include <stdexcept>

// CAN frame definitions from: https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1827176527/CAN+Frames
namespace CAN
{

using CanFrame   = struct canfd_frame;
using RawDataBuf = std::array<uint8_t, CANFD_MAX_DLEN>;

enum CanId : uint32_t {
    BMS_P_DATA_FRAME_1     = 0x31,
    BMS_P_DATA_FRAME_2     = 0x32,
    SAIL_WSM_CMD_FRAME_1   = 0x60,
    SAIL_WSM_CMD_FRAME_2   = 0x61,
    SAIL_ENCD_DATA_FRAME   = 0x62,
    SAIL_WSM_DATA_FRAME_1  = 0x63,
    SAIL_WSM_DATA_FRAME_2  = 0x64,
    SAIL_WIND_DATA_FRAME_1 = 0x65,
    SAIL_NAV_CMD_FRAME     = 0x66,
    RUDR_CMD_FRAME         = 0x70,
    RUDR_DATA_FRAME_1      = 0x71,
    RUDR_DATA_FRAME_2      = 0x72,
    PATH_GPS_DATA_FRAME_1  = 0x80,
    PATH_GPS_DATA_FRAME_2  = 0x81,
    PATH_GPS_DATA_FRAME_3  = 0x82,
    PATH_GPS_DATA_FRAME_4  = 0x83,
    PATH_WIND_DATA_FRAME   = 0x84,
};

static const std::map<CanId, std::string> CanDescription{
  {BMS_P_DATA_FRAME_1, "Battery 1 data"},  // Dummy comment to pad the map for prettier formatting
  {BMS_P_DATA_FRAME_2, "Battery 2 data"},
  {SAIL_WSM_CMD_FRAME_1, "Main sail command"},
  {SAIL_WSM_CMD_FRAME_2, "Jib Sail command"},
  {SAIL_ENCD_DATA_FRAME, "Sail encoder data"},
  {SAIL_WSM_DATA_FRAME_1, "Main sail data"},
  {SAIL_WSM_DATA_FRAME_2, "Jib sail data"},
  {SAIL_WIND_DATA_FRAME_1, "Mast wind sensor"},
  {SAIL_NAV_CMD_FRAME, "Nav light commands"},
  {RUDR_CMD_FRAME, "Rudder commands (BOTH RUDDERS)"},
  {RUDR_DATA_FRAME_1, "Port rudder data,"},
  {RUDR_DATA_FRAME_2, "Starboard rudder data"},
  {PATH_GPS_DATA_FRAME_1, "GPS latitude"},
  {PATH_GPS_DATA_FRAME_2, "GPS longitude"},
  {PATH_GPS_DATA_FRAME_3, "GPS other data"},
  {PATH_GPS_DATA_FRAME_4, "GPS time reporting (ex. day of the year)"},
  {PATH_WIND_DATA_FRAME, "Hull wind sensor"}};

/**
 * Custom exception for when an attempt is made to construct a CAN object with a mismatched ID
 *
 */
class CanIdMismatchException : public std::exception
{
public:
    CanIdMismatchException(std::span<const CanId> valid_ids, const canid_t & received);

    using std::exception::what;  // Needed to silence virtual function overload error
    const char * what();

private:
    std::string msg;
};

struct CanBase
{
    CanId id_;

    CanBase()            = delete;
    CanBase(CanFrame cf) = delete;
    CanBase(std::span<const CanId> valid_ids, CanId id);
    friend std::ostream & operator<<(std::ostream & os, const CanBase & can);
};

struct Battery : public CanBase
{
    static constexpr std::array<CanId, 2> BATTERY_IDS = {BMS_P_DATA_FRAME_1, BMS_P_DATA_FRAME_2};
    // Note: Each BMS battery is comprised of multiple battery cells
    float volt_;      // Average voltage of cells in the battery
    float curr_;      // Current - positive means charging and negative means discharging (powering the boat)
    float volt_max_;  // Maximum voltage of cells in the battery pack (unused)
    float volt_min_;  // Minimum voltage of cells in the battery pack (unused)

    explicit Battery(CanFrame cf);
    friend std::ostream & operator<<(std::ostream & os, const Battery & can);
};

}  // namespace CAN
