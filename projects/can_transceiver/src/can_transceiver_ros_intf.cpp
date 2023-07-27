#include <memory>

#include "can_frame_parser.h"
#include "can_transceiver.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "ros_info.h"
#include "std_msgs/msg/string.hpp"

class CanTransceiverIntf : public rclcpp::Node
{
public:
    explicit CanTransceiverIntf(std::shared_ptr<CanTransceiver> can_trns)
    : Node("can_transceiver_intf_node"), can_trns_(can_trns)
    {
        static constexpr int  ROS_Q_SIZE     = 5;
        static constexpr auto TIMER_INTERVAL = std::chrono::milliseconds(500);
        pub_   = this->create_publisher<std_msgs::msg::String>(PLACEHOLDER_TOPIC_0_TOPIC, ROS_Q_SIZE);
        timer_ = this->create_wall_timer(TIMER_INTERVAL, std::bind(&CanTransceiverIntf::pub_cb, this));
        sub_   = this->create_subscription<std_msgs::msg::String>(
          PLACEHOLDER_TOPIC_1_TOPIC, ROS_Q_SIZE, std::bind(&CanTransceiverIntf::sub_cb, this, std::placeholders::_1));
    }

private:
    std::shared_ptr<CanTransceiver> can_trns_;
    // Publishing timer
    rclcpp::TimerBase::SharedPtr timer_;
    // String is a placeholder pub and sub msg type - we will definitely define custom message types
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    /**
     * @brief Callback function to publish to onboard ROS network
     *
     */
    void pub_cb(/* placeholder */)
    {
        std::string recent_sensors = can_trns_->getRecentSensors();
        auto        msg            = std_msgs::msg::String();
        msg.data                   = recent_sensors;
        pub_->publish(msg);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network
     * 
     */
    void sub_cb(std_msgs::msg::String /* placeholder */) { can_trns_->onNewCmd(Placeholder0); }
};

//===========================================================================================
// Simulation Interface Components
//===========================================================================================
// [Simulator]<->[CAN Simulation Interface]<->[Virtual CAN bus]<->[CAN Transceiver]

// Refer to https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1768849494/Simulation+Interface#Interfaces

/**
 * Implementation of CAN Transceiver that interfaces with the simulator
 * This replaces the CAN bus during testing to interface with simulator
 *
 * Receives info from ROS node with topic, specified by controls team
 * Refer to https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1785790595/Boat+Simulator+Design+Specification
 * 
 * Receives from ROS topic, /Simulator. Controls team sends to that
 * This is done by subscribing to ROS node /Simulator
 */

class CanSimIntf : public CanTransceiver, public rclcpp::Node
{
    // There is no timer because subscriber will respond to whatever data is published to the topic /Simulator
public:
    CanSimIntf() : Node("Simulator")
    {
        int simulatorData = 0;  //Placeholder

        subscription_ = this->create_subscription<std_msgs::msg::String>(
          // Topic is called /Simulator
          "Simulator", simulatorData, std::bind(&CanSimIntf::topic_callback, this, std::placeholders::_1));
    }

private:
    // Receives string message data over topic and writes to RCLCPP_INFO macro
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

//===========================================================================================
// Main
//===========================================================================================

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    return 0;
}
