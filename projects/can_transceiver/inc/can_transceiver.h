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
    * @brief Construct a new Canbus Intf object and start listening for incoming data on a new thread
    *
    * @param can_inst
    */
    CanTransceiver();

    explicit CanTransceiver(int fd);

    /**
     * @brief Destroy the Canbus Intf object
     *
     */
    ~CanTransceiver();

    /**
     * @brief Send a command to hardware
     *
     * @param frame command frame to send
     */
    void send(CAN::CanFrame frame) const;
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

private:
    // CAN socket this instance is attached to
    int sock_desc_;
    // mutable keyword required for std::lock_guard
    mutable std::mutex can_mtx_;

    // Thread that listens to CAN
    std::thread receive_thread_;
    // Flag to tell the receive_thread_ to stop
    bool shutdown_flag_ = false;

    std::map<CAN::CanId, std::function<void(const CAN::CanFrame &)>> read_callbacks_;

    /**
     * @brief Retrieve latest incoming CAN frame from hardware and process it
     *
     */
    void receive();
    /**
     * @brief Call on receiving a new CAN data frame from hardware/simulator
     *
     * @param frame received CAN data frame
     */
    void onNewCanData(CAN::CanFrame frame) const;
};
