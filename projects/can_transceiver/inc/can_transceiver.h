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
    virtual ~CanTransceiver();

    /**
     * @brief Call when a new command (ex. rudder command) needs to be executed
     *        Passes the command down to the hardware/simulator
     *
     */
    void onNewCmd(CAN::CanFrame cmd_frame);

    /**
     * @brief Retrieve the most recent set of sensors data
     *
     * @return Data in some format - string is just a placeholder; DO NOT USE STRING IN ACTUAL IMPLEMENTATION
     */
    std::string getRecentSensors();

    void registerCanCb(std::pair<CAN::CanId, std::function<void(CAN::CanFrame)>> cb_kvp);

    void registerCanCbs(
      const std::initializer_list<std::pair<CAN::CanId, std::function<void(CAN::CanFrame)>>> & cb_kvps);

    /**
     * @brief Send a command to the hardware/simulator
     *
     * @param frame Command frame to send
     */
    virtual void send(CAN::CanFrame frame) const = 0;

protected:
    /**
     * @brief Call on receiving a new CAN data frame from hardware/simulator
     *
     * @param frame received CAN data frame
     */
    void onNewCanData(CAN::CanFrame frame) const;

private:
    /**
     * @brief Retrieve latest incoming data from hardware/simulator and process it
     *
     */
    virtual void receive() = 0;

    std::map<CAN::CanId, std::function<void(const CAN::CanFrame &)>> read_callbacks_;
};

/**
 * Implementation of CAN Transceiver that interfaces with CAN hardware
 *
 */
class CanbusTransceiver : public CanTransceiver
{
public:
    /**
    * @brief Construct a new Canbus Intf object and start listening for incoming data on a new thread
    *
    * @param can_inst
    */
    CanbusTransceiver();

    explicit CanbusTransceiver(int fd);

    /**
     * @brief Destroy the Canbus Intf object
     *
     */
    ~CanbusTransceiver();

    /**
     * @brief Send a command to hardware
     *
     * @param frame command frame to send
     */
    void send(CAN::CanFrame frame) const;

private:
    // Thread that listens to CAN
    std::thread        receive_thread_;
    bool               shutdown_flag_ = false;
    mutable std::mutex can_mtx_;  // mutable keyword required for std::lock_guard

    // CAN socket this instance is attached to
    int sock_desc_;

    /**
     * @brief Retrieve latest incoming CAN frame from hardware and process it
     *
     */
    void receive();
};

/**
 * Implementation of CAN Transceiver that interfaces with the simulator
 *
 */
class CanSimTransceiver : public CanTransceiver
{
public:
    CanSimTransceiver();
    void send(CAN::CanFrame frame) const;

private:
    void receive();
};
