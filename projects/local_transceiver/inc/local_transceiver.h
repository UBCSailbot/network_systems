#pragma once

#include <mutex>
#include <string>

#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "custom_interfaces/msg/ais_ships.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "rclcpp/node.hpp"
#include "sensors.pb.h"
#include "std_msgs/msg/string.hpp"

namespace msg = custom_interfaces::msg;

/**
 * Implementation of Local Transceiver that operates through a serial interface
 */
class LocalTransceiver
{
    /**
     * Buffer class to synchronize sensors updates and transmissions
     */
    class SensorBuf
    {
    public:
        /**
         * @brief Construct a new Sensor Buf object
         *
         */
        SensorBuf();

        /**
         * @brief Update the buffer with new GPS data
         *
         * @param gps custom_interfaces gps object
         */
        void updateSensor(msg::GPS gps);

        /**
         * @brief Update the buffer with new AIS Ships data
         *
         * @param ships custom_interfaces AISShips object
         */
        void updateSensor(msg::AISShips ships);

        // TODO(Jng468): Add other sensors

        /**
         * @brief Get a copy of the sensors object
         *
         * @return Copy of sensors_
         */
        Polaris::Sensors sensors();

    private:
        Polaris::Sensors sensors_;  // Underlying Sensors object
        std::mutex       buf_mtx_;  // Mutex to synhronize access to sensors_
    };

public:
    /**
     * @brief Construct a new Local Transceiver object and connect it to a serial port
     *
     * @param port_name serial port (ex. /dev/ttyS0)
     * @param baud_rate baud rate of the serial port
     */
    LocalTransceiver(const std::string & port_name, uint32_t baud_rate);

    /**
     * @brief Destroy the Local Transceiver object and close the serial port
     *
     */
    ~LocalTransceiver();

    /**
     * @brief Callback function for when new sensor data is received from the ROS network on Polaris
     *
     * @tparam T of type custom_interfaces::msg::T
     * @param sensor new sensor data
     */
    template <typename T>
    void onNewSensorData(T sensor);

    /**
     * @brief Send current data to the serial port and to the remote server
     *
     * @return true  on success
     * @return false on failure
     */
    bool send();

    /**
     * @brief Send a debug command and return the output
     *
     * @param cmd string to send to the serial port
     * @return output of the sent cmd
     */
    std::string debugSend(const std::string & cmd);

    /**
     * @brief Retrieve the latest message from the remote server via the serial port
     *
     * @return The message as a binary string
     */
    std::string receive();

private:
    // boost io service - required for boost::asio operations
    boost::asio::io_service io_;
    // serial port data where is sent and received
    boost::asio::serial_port serial_;
    // mutex to synchronize access to the serial port
    std::mutex serial_mtx_;
    // Buffer for sensors to be sent to the satellite
    SensorBuf sensor_buf_;

    /**
     * @brief Send a command to the serial port
     *
     * @param cmd command to send
     */
    void send(const std::string & cmd);

    /**
     * @brief Formats binary data to be sent to the satellite according to the AT command specification
     *
     * @param data data binary string to be sent
     * @return Formatted message to be written to serial
     */
    static std::string createOutMsg(const std::string & data);

    /**
     * @brief Parse the message received from the remote server
     *
     * @param msg message received from the remote server
     * @return the data byte string payload from the message
     */
    static std::string parseInMsg(const std::string & msg);

    /**
     * @brief Read a line from serial
     *
     * @return line
     */
    std::string readLine();

    /**
     * @brief Check that the last command sent to serial was valid
     *
     * @return true  if valid
     * @return false if invalid
     */
    bool checkOK();
};
