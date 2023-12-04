#pragma once

#include <linux/can.h>

#include <rclcpp/rclcpp.hpp>
#include <string>

#include "can_frame_parser.h"

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
class CanSimTransceiver : public CanTransceiver
{
    void receive();
};
