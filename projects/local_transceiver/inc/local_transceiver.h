#pragma once

#include <mutex>
#include <string>

#include "boost/asio/io_service.hpp"
#include "boost/asio/serial_port.hpp"
#include "rclcpp/node.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * Abstract Local Transceiver Class
 * Handles transmission and reception of data to and from the remote server
 *
 */
class LocalTransceiver
{
public:
    /**
    * @brief Destroy the Local Transceiver object
    * 
    */
    virtual ~LocalTransceiver() = 0;

    /**
     * @brief Call when new data is received from the ROS network on the boat.
     *        Serializes the data and sends it off to the remote server
     * 
     */
    void onNewSensorData(/* some datatype */);

    /**
     * @brief Get the latest data from the remote server
     * 
     * @return ROS msg to send back to the boat ROS network (string is a placeholder)
     */
    std::string getRemoteData();

protected:
    /**
     * @brief Formats the data to have the necessary payload headers for our transmission method
     * 
     * @param data data byte string to be sent
     * @return Formatted message
     */
    static std::string formatMsg(const std::string & data);

    /**
     * @brief Parse the message received from the remote server
     * 
     * @param msg message received from the remote server
     * @return the data byte string payload from the message
     */
    static std::string parseMsg(const std::string & msg);

    /**
     * @brief Send a data byte string to our transmission medium
     * 
     * @param data byte string to send
     * @return true  on success
     * @return false on failure
     */
    virtual bool send(const std::string & data) = 0;

    /**
     * @brief Retrieve the latest message from our reception medium
     * 
     * @return The data byte stream of the message payload
     */
    virtual std::string receive() = 0;
};

/**
 * Implementation of Local Transceiver that operates through a serial interface
 */
class HwLocalTransceiver : public LocalTransceiver
{
public:
    /**
     * @brief Construct a new Hw Local Transceiver object
     * 
     * @param serial_id  serial port (ex. /dev/ttyS0)
     */
    explicit HwLocalTransceiver(const std::string & serial_id);

    /**
     * @brief Destroy the Hw Local Transceiver object
     * 
     */
    ~HwLocalTransceiver();

    /**
     * @brief Send data to the serial port and onto the remote server
     * 
     * @param data byte string to send
     * @return true  on success
     * @return false on failure
     */
    bool send(const std::string & data);

    /**
     * @brief Retrieve the latest message from the remote server via the serial port
     * 
     * @return The data byte stream of the message payload
     */
    std::string receive();

private:
    // boost io service - required for boost::asio operations
    boost::asio::io_service io_;
    // serial port data where is sent and received
    boost::asio::serial_port serial_;
    // mutex to synchronize access to the serial port
    std::mutex serial_mtx_;
};

/**
 * Mock version of the Local Transceiver
 * Sends and receives data via ROS to a locally ran Mock Remote Transceiver
 * 
 */
class MockLocalTransceiver : public LocalTransceiver, public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Mock Local Transceiver object
     * 
     */
    explicit MockLocalTransceiver();

    /**
     * @brief Destroy the Mock Local Transceiver object
     * 
     */
    ~MockLocalTransceiver();

    /**
     * @brief Send data to the ROS network and onto Mock Remote Transceiver
     * 
     * @param data byte string to send
     * @return true  on success
     * @return false on failure
     */
    bool send(const std::string & data);

    /**
     * @brief Retrieve the latest message from the Mock Remote Transceiver via ROS 
     * 
     * @return The data byte stream of the message payload
     */
    std::string receive();

private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    // Latest message published by the Mock Remote Transceiver
    std::string latest_rcvd_msg_;
    // Callback function to subscribe to the MOCK_REMOTE_TO_LOCAL_TRANSCEIVER_TOPIC
    void sub_callback(std_msgs::msg::String::SharedPtr msg);
};
