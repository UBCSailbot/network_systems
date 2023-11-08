#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "can_frame_parser.h"
#include "can_transceiver.h"
#include "custom_interfaces/msg/batteries.hpp"
#include "custom_interfaces/msg/generic_sensors.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "custom_interfaces/msg/helper_generic_sensor.hpp"
#include "custom_interfaces/msg/wind_sensor.hpp"
#include "custom_interfaces/msg/wind_sensors.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "ros_info.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

constexpr int QUEUE_SIZE        = 10;
constexpr int PLACEHOLDER_VALUE = 42;  // Placeholder value for debugging or testing

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
// Simulation Interface
//===========================================================================================
/* Refer to https://ubcsailbot.atlassian.net/wiki/spaces/prjt22/pages/1768849494/Simulation+Interface#Interfaces
 * Handles publishing and subscribing to certain ROS topics
 */
class CanSimIntf : public rclcpp::Node
{
public:
    CanSimIntf()  //Our node which subs to topics
    : Node("CanSimIntf")
    {
        // Subscriber
        // There is no timer because subscriber will respond to whatever data is published to the topic /Simulator
        // Topic: mock_gps
        subscriptionGPS_ = this->create_subscription<custom_interfaces::msg::GPS>(
          "mock_gps", QUEUE_SIZE, std::bind(&CanSimIntf::gps_callback, this, std::placeholders::_1));

        // Topic: mock_wind_sensors
        subscriptionWindSensor_ = this->create_subscription<custom_interfaces::msg::WindSensors>(
          "mock_wind_sensors", QUEUE_SIZE, std::bind(&CanSimIntf::wind_callback, this, std::placeholders::_1));

        // Publisher
        publisherWindSensors_ = this->create_publisher<custom_interfaces::msg::WindSensors>("wind_sensors", QUEUE_SIZE);
        publisherWindSensor_ =
          this->create_publisher<custom_interfaces::msg::WindSensor>("filtered_wind_sensor", QUEUE_SIZE);
        publisherGPS_ = this->create_publisher<custom_interfaces::msg::GPS>("gps", QUEUE_SIZE);
        publisherGenericSensors_ =
          this->create_publisher<custom_interfaces::msg::GenericSensors>("data_sensors", QUEUE_SIZE);
        publisherBatteries_ = this->create_publisher<custom_interfaces::msg::Batteries>("batteries", QUEUE_SIZE);
        // Timer with 500ms delay
        timer_ = this->create_wall_timer(500ms, std::bind(&CanSimIntf::timer_callback, this));
    }

private:
    // Subscriber
    void gps_callback(const custom_interfaces::msg::GPS::SharedPtr msg) const {}
    void wind_callback(const custom_interfaces::msg::WindSensors::SharedPtr msg) const {}

    // Publisher
    void timer_callback()
    {
        // ** GPS **
        custom_interfaces::msg::GPS gpsPubData;
        gpsPubData.heading.heading   = PLACEHOLDER_VALUE;
        gpsPubData.speed.speed       = PLACEHOLDER_VALUE;
        gpsPubData.lat_lon.latitude  = PLACEHOLDER_VALUE;
        gpsPubData.lat_lon.longitude = PLACEHOLDER_VALUE;
        // Publishes msg
        publisherGPS_->publish(gpsPubData);

        // ** Wind Sensor (WSensor)**
        custom_interfaces::msg::WindSensor WSensorData;
        WSensorData.speed.speed = PLACEHOLDER_VALUE;
        WSensorData.direction   = PLACEHOLDER_VALUE;
        // Publishes msg
        publisherWindSensor_->publish(WSensorData);

        // ** Wind Sensors (WSensors)**
        custom_interfaces::msg::WindSensors WSensorsData;
        WSensorsData.wind_sensors[0].direction   = PLACEHOLDER_VALUE;
        WSensorsData.wind_sensors[0].speed.speed = PLACEHOLDER_VALUE;
        // Publishes msg
        publisherWindSensors_->publish(WSensorsData);

        // ** Batteries **
        custom_interfaces::msg::Batteries WBatteriesData;
        WBatteriesData.batteries[0].voltage = PLACEHOLDER_VALUE;
        WBatteriesData.batteries[0].current = PLACEHOLDER_VALUE;
        // Publishes msg
        publisherBatteries_->publish(WBatteriesData);

        // ** Generic Sensors **
        custom_interfaces::msg::HelperGenericSensor HelperGenSensorData;
        custom_interfaces::msg::GenericSensors      GenSensorData;
        HelperGenSensorData.id   = 0x0;  //uint8
        HelperGenSensorData.data = 0x0;  //uint64
        // Publishes msg
        GenSensorData.generic_sensors.push_back(HelperGenSensorData);
        publisherGenericSensors_->publish(GenSensorData);
    }

    // Field Operations

    // Publisher Field Declarations
    rclcpp::Publisher<custom_interfaces::msg::WindSensors>::SharedPtr    publisherWindSensors_;
    rclcpp::Publisher<custom_interfaces::msg::WindSensor>::SharedPtr     publisherWindSensor_;
    rclcpp::Publisher<custom_interfaces::msg::GPS>::SharedPtr            publisherGPS_;
    rclcpp::Publisher<custom_interfaces::msg::GenericSensors>::SharedPtr publisherGenericSensors_;
    rclcpp::Publisher<custom_interfaces::msg::Batteries>::SharedPtr      publisherBatteries_;

    // Subscriber Field Declarations
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr         subscriptionGPS_;
    rclcpp::Subscription<custom_interfaces::msg::WindSensors>::SharedPtr subscriptionWindSensor_;

    // Timer object to allow our CamSimIntfFeedback node to perform action at x rate
    rclcpp::TimerBase::SharedPtr timer_;
};

//===========================================================================================
// Main
//===========================================================================================
// Run with:
//$ ros2 run network_systems can_transceiver
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanSimIntf>());
    rclcpp::shutdown();
    return 0;
}
