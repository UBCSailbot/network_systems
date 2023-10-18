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



/***** Bounds for Battery ******/
constexpr float VOLT_LBND = 0.5;       // arbitrary number
constexpr float VOLT_UBND = 250.0;     // arbitrary number
constexpr float CURRENT_LBND = -200.0; // arbitrary number - confirm? :)
constexpr float CURRENT_UBND = 200.0;  // arbitrary number - confirm? :)

/***** Bounds for Wind Sensor ******/
constexpr int DIRECTION_LBND = -180;
constexpr int DIRECTION_UBND = 179;
