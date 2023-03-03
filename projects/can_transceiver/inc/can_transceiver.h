#pragma once

#include <linux/can.h>

#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>

#include "can_frame_parser.h"
#include "rclcpp/rclcpp.hpp"

using std::placeholders::_1;

/**
 * Abstract CAN Transceiver Class
 * Handles transmission and reception of data to and from the hardware/simulator
 * 
 */
class CanTransceiver
{
public:
    /**
    * @brief Destroy the Can Transceiver object
    * 
    */
    virtual ~CanTransceiver() = 0;

    /**
     * @brief Call when a new command (ex. rudder command) needs to be executed
     *        Passes the command down to the hardware/simulator
     * 
     */
    void onNewCmd(CanId id /*, other data fields... */);

    /**
     * @brief Retrieve the most recent set of sensors data
     * 
     * @return Data in some format - string is just a placeholder; DO NOT USE STRING IN ACTUAL IMPLEMENTATION
     */
    std::string getRecentSensors();

protected:
    // Buffer where recent data is stored - DO NOT USE VOID POINTER IN ACTUAL IMPLEMENTATION
    void * sensor_buf_;
    /**
     * @brief Retrieve latest incoming data from hardware/simulator and process it
     * 
     */
    virtual void receive() = 0;

    /**
     * @brief Send a command to the hardware/simulator
     * 
     * @param frame Command frame to send
     */
    virtual void send(const CanFrame & frame) const = 0;

    /**
     * @brief Call on receiving a new CAN data frame from hardware/simulator
     * 
     * @param frame received CAN data frame
     */
    void onNewCanData(const CanFrame & frame);
};

/**
 * Implementation of CAN Transceiver that interfaces with CAN hardware
 * 
 */
class CanbusIntf : public CanTransceiver
{
public:
    /**
    * @brief Construct a new Canbus Intf object and start listening for incoming data on a new thread
    * 
    * @param can_inst 
    */
    explicit CanbusIntf(const std::string & can_inst);

    /**
     * @brief Destroy the Canbus Intf object
     * 
     */
    ~CanbusIntf();

private:
    // Thread that listens to CAN
    std::thread receive_thread_;

    // CAN socket this instance is attached to
    int sock_desc_;

    /**
     * @brief Retrieve latest incoming CAN frame from hardware and process it
     * 
     */
    void receive();

    /**
     * @brief Send a command to hardware
     * 
     * @param frame command frame to send
     */
    void send(const CanFrame & frame) const;
};

/**
 * Implementation of CAN Transceiver that interfaces with the simulator
 * 
 */
//===========================================================================================
// Simulation Interface Components
//===========================================================================================
// [Simulator]<->[CAN Simulation Interface]<->[Virtual CAN bus]<->[CAN Transceiver]

// Refer to https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1768849494/Simulation+Interface#Interfaces

/**
 * Implementation of CAN Transceiver that interfaces with the simulator
 * This replaces the CAN bus during testing to interface with simulator
 */
// Receives info from ROS node with topic, specified by controls team
// Refer to https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1785790595/Boat+Simulator+Design+Specification

// Subscribes to topic and gets this information:
// Latitude (degrees)
// Longitude (degrees)
// Boat speed (knots)
// Boat accel (knots/s)
// Boat Bearing (Degrees)
// Wind speed (Knots)
// Wind Bearing (Degrees)

// Receives from ROS topic, /Simulator. Controls team sends to that
// This is done by subscribing to ROS node /Simulator
class CanSimIntf : public CanTransceiver, public rclcpp::Node
{
    // There is no timer because subscriber will respond to whatever data is published to the topic /Simulator
public:
    CanSimIntf() : Node("Simulator")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
          // Topic is called /Simulator
          "Simulator", 10, std::bind(&CanSimIntf::topic_callback, this, _1));  //TO CHANGE in future. Probably not 10
    }

private:
    // Receives string message data over topic and writes to RCLCPP_INFO macro
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * This will interface with virtual CAN bus 
 * Look into in future on how to set up.
 */
