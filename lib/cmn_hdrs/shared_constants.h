#pragma once

#include <string>

/**
 * ROS argument value for system mode. An enum would be a better way of representing a binary choice between the two
 * options, but since strings are not integral types they cannot be made into enums.
 */
namespace SYSTEM_MODE
{
static const std::string PROD = "production";
static const std::string DEV  = "development";
};  // namespace SYSTEM_MODE

constexpr unsigned int MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES = 270;
constexpr unsigned int MAX_REMOTE_TO_LOCAL_PAYLOAD_SIZE_BYTES = 340;

constexpr int NUM_BATTERIES    = 2;
constexpr int NUM_WIND_SENSORS = 2;

/****** Upper and lower bounds ******/

/***** Bounds for Latitude and Longitude ******/
constexpr float LAT_LBND = -90.0;
constexpr float LAT_UBND = 90.0;
constexpr float LON_LBND = -180.0;
constexpr float LON_UBND = 180.0;

/***** Bounds for Speed ******/
constexpr float SPEED_LBND = -10.0;  // Placeholder number
constexpr float SPEED_UBND = 10.0;   // Placeholder number

/***** Bounds for Heading ******/
constexpr float HEADING_LBND = 0.0;
constexpr float HEADING_UBND = 360.0;

// boat rotation
constexpr float ROT_LBND = -360.0;
constexpr float ROT_UBND = 360;
// boat dimension
constexpr float DIMENSION_LBND = 0;
constexpr float DIMENSION_UBND = 650.0;

/***** Bounds for Battery ******/
constexpr float VOLT_LBND    = 0.5;     // Placeholder number
constexpr float VOLT_UBND    = 250.0;   // Placeholder number
constexpr float CURRENT_LBND = -200.0;  // Placeholder number
constexpr float CURRENT_UBND = 200.0;   // Placeholder number

/***** Bounds for Wind Sensor ******/
constexpr int DIRECTION_LBND = -180;
constexpr int DIRECTION_UBND = 179;
