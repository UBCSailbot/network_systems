#include <custom_interfaces/msg/detail/gps__struct.hpp>
#include <custom_interfaces/msg/detail/wind_sensor__struct.hpp>
#include <custom_interfaces/msg/detail/wind_sensors__struct.hpp>
#include <memory>

#include "can_frame_parser.h"
#include "can_transceiver.h"
#include "custom_interfaces/msg/gps.hpp"
#include "custom_interfaces/msg/wind_sensors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "ros_info.h"
#include "std_msgs/msg/string.hpp"

#define QUEUE_SIZE 10

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
// Simulation Interface Component (FLow Up)
//===========================================================================================
/* Refer to https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1768849494/Simulation+Interface#Interfaces
 * Control Simulator -> CanSimIntf() -> CAN Transceiver
 * Also subscribes to simulator over ROS
 */
class CanSimIntf : public CanTransceiver, public rclcpp::Node
{
    // There is no timer because subscriber will respond to whatever data is published to the topic /Simulator
public:
    // Our node which publishes
    // Node can publish to any number of topics

    CanSimIntf()  //Our node which subs to topics
    : Node("CanSimIntfSubscriber")
    {
        // Topic: mock_gps
        subscriptionGPS_ = this->create_subscription<custom_interfaces::msg::GPS>(
          "mock_gps", QUEUE_SIZE, std::bind(&CanSimIntf::gps_callback, this, std::placeholders::_1));

        // Topic: mock_wind_sensors
        subscriptionWindSensor_ = this->create_subscription<custom_interfaces::msg::WindSensor>(
          "mock_wind_sensors", QUEUE_SIZE, std::bind(&CanSimIntf::wind_callback, this, std::placeholders::_1));
    }

private:
    // Receives string message data over topic and writes to RCLCPP_INFO macro
    // When moment message is available in queue: prints statement in log
    void gps_callback(const custom_interfaces::msg::GPS::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->heading.heading);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->lat_lon.latitude);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->lat_lon.longitude);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->speed.speed);
    }
    void wind_callback(const custom_interfaces::msg::WindSensor::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->direction);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->speed.speed);
    }
    // Field declaration: Subscription Subscribers
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr subscriptionGPS_;
    rclcpp::Subscription<custom_interfaces::msg::WindSensor>::SharedPtr subscriptionWindSensor_;

};

//===========================================================================================
// Main
//===========================================================================================
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    return 0;
}
