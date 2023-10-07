#include <cstdio>
#include <custom_interfaces/msg/detail/batteries__struct.hpp>
#include <custom_interfaces/msg/detail/generic_sensors__struct.hpp>
#include <custom_interfaces/msg/detail/gps__struct.hpp>
#include <custom_interfaces/msg/detail/wind_sensor__struct.hpp>
#include <custom_interfaces/msg/detail/wind_sensors__struct.hpp>
#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <thread>

#include "can_frame_parser.h"
#include "can_transceiver.h"

#include "custom_interfaces/msg/gps.hpp"
#include "custom_interfaces/msg/wind_sensors.hpp"
#include "custom_interfaces/msg/batteries.hpp"
#include "custom_interfaces/msg/generic_sensors.hpp"
#include "custom_interfaces/msg/wind_sensor.hpp"

#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "ros_info.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

#define QUEUE_SIZE 10
#define WIND_SENSOR_BOUND 2
#define PLACEHOLDER_VALUE 42 // Placeholder value for debugging or testing

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
        publisherWindSensor_ = this->create_publisher<custom_interfaces::msg::WindSensor>("filtered_wind_sensor", QUEUE_SIZE);
        publisherGPS_ = this->create_publisher<custom_interfaces::msg::GPS>("gps", QUEUE_SIZE);
        publisherGenericSensors_ = this->create_publisher<custom_interfaces::msg::GenericSensors>("data_sensors", QUEUE_SIZE);
        publisherBatteries_ = this->create_publisher<custom_interfaces::msg::Batteries>("batteries", QUEUE_SIZE);
        // Timer with 500ms delay
        timer_     = this->create_wall_timer(500ms, std::bind(&CanSimIntf::timer_callback, this));
    }

private:

// Subscriber
// When moment message is available in queue: prints statement in log
// Reminder: If publisher node (boat_simulator isn't running, this will not print.)
    void gps_callback(const custom_interfaces::msg::GPS::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->heading.heading); //TO-DO: Combine into 1 print
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->lat_lon.latitude);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->lat_lon.longitude);
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->speed.speed);
    }
    void wind_callback(const custom_interfaces::msg::WindSensors::SharedPtr msg) const
    {
        for(int i=0;i<WIND_SENSOR_BOUND;++i){
        RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->wind_sensors[i].speed.speed); //TO-DO: Combine into 1 ()
        RCLCPP_INFO(this->get_logger(), "I heard: '%i'", msg->wind_sensors[i].direction);
        }
    }

// Publisher
    void timer_callback()
    {
        // ** GPS **
        auto gpsPubData = custom_interfaces::msg::GPS();
        gpsPubData.heading.heading = PLACEHOLDER_VALUE;
        gpsPubData.speed.speed = PLACEHOLDER_VALUE;
        gpsPubData.lat_lon.latitude = PLACEHOLDER_VALUE;
        gpsPubData.lat_lon.longitude = PLACEHOLDER_VALUE;
        // Log
        RCLCPP_INFO(this->get_logger(), "GPS Pub: '%f' '%f' '%f' '%f' ",
        gpsPubData.heading.heading, gpsPubData.speed.speed,  gpsPubData.lat_lon.latitude, gpsPubData.lat_lon.longitude);
        // Publishes msg
        publisherGPS_->publish(gpsPubData);

        // ** Wind Sensor (WSensor)**
        auto WSensorData = custom_interfaces::msg::WindSensor();
        WSensorData.speed.speed = PLACEHOLDER_VALUE;
        WSensorData.direction = PLACEHOLDER_VALUE;
        // Log
        RCLCPP_INFO(this->get_logger(), "Wind Sensor Pub: '%f' '%d'",
        WSensorData.speed.speed, WSensorData.direction);
        // Publishes msg
        publisherWindSensor_->publish(WSensorData);

        // ** Wind Sensors (WSensors)**
        auto WSensorsData = custom_interfaces::msg::WindSensors();
        // Note: for loop causes stuck here, maybe have to do separate timer_callback
        // OR just hard code [1]. [2], .... etc.
        WSensorsData.wind_sensors[0].direction = PLACEHOLDER_VALUE;
        WSensorsData.wind_sensors[0].speed.speed = PLACEHOLDER_VALUE;
        RCLCPP_INFO(this->get_logger(), "Wind Sensors Pub: '%d' '%f'",
        WSensorsData.wind_sensors[0].direction, WSensorsData.wind_sensors[0].speed.speed);
        // Publishes msg
        publisherWindSensors_->publish(WSensorsData);

        // ** Batteries **
        auto WBatteriesData = custom_interfaces::msg::Batteries();
        WBatteriesData.batteries[0].voltage = PLACEHOLDER_VALUE;
        WBatteriesData.batteries[0].current = PLACEHOLDER_VALUE;
        RCLCPP_INFO(this->get_logger(), "Batteries Pub: '%f' '%f'",
        WBatteriesData.batteries[0].voltage, WBatteriesData.batteries[0].current);
        // Publishes msg
        publisherBatteries_->publish(WBatteriesData);

        // // ** Generic Sensors **
        // auto GenSensorData = custom_interfaces::msg::GenericSensors();
        // GenSensorData.generic_sensors[0].id = 0x0; //uint8
        // GenSensorData.generic_sensors[0].data = 0x0; //uint64
        // // Publishes msg
        // publisherGenericSensors_->publish(GenSensorData);

        //I think '%lu' is causing seg err since data is uint64, and id is uint8. Removed print for now.
        // Debug why segmentation error for this. Guessing .id and .data cannot be done like this.

    }

// Field Operations

    // Publisher Field Declarations
    rclcpp::Publisher<custom_interfaces::msg::WindSensors>::SharedPtr publisherWindSensors_;
    rclcpp::Publisher<custom_interfaces::msg::WindSensor>::SharedPtr publisherWindSensor_;
    rclcpp::Publisher<custom_interfaces::msg::GPS>::SharedPtr publisherGPS_;
    rclcpp::Publisher<custom_interfaces::msg::GenericSensors>::SharedPtr publisherGenericSensors_;
    rclcpp::Publisher<custom_interfaces::msg::Batteries>::SharedPtr publisherBatteries_;

    // Subscriber Field Declarations
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr subscriptionGPS_;
    rclcpp::Subscription<custom_interfaces::msg::WindSensors>::SharedPtr subscriptionWindSensor_;

    // Variable to count number of msgs published
    //size_t                                              count_;

    // Timer object to allow our CamSimIntfFeedback node to perform action at x rate
    rclcpp::TimerBase::SharedPtr                        timer_;

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
