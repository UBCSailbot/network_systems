#pragma once

constexpr unsigned int MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES = 270;
constexpr unsigned int MAX_REMOTE_TO_LOCAL_PAYLOAD_SIZE_BYTES = 340;

/****** Upper and lower bounds ******/
// latitude and longitude
constexpr float LAT_LBND = -90.0;
constexpr float LAT_UBND = 90.0;
constexpr float LON_LBND = -180.0;
constexpr float LON_UBND = 180.0;
// boat speed
constexpr float SPEED_LBND = -10.0;  // arbitrary number
constexpr float SPEED_UBND = 10.0;   //arbitrary number
// boat heading
constexpr float HEADING_LBND = 0.0;
constexpr float HEADING_UBND = 360.0;
