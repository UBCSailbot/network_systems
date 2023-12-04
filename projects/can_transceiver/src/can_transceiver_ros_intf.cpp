#include <chrono>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/batteries.hpp>
#include <custom_interfaces/msg/can_sim_to_boat_sim.hpp>
#include <custom_interfaces/msg/desired_heading.hpp>
#include <custom_interfaces/msg/generic_sensors.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <custom_interfaces/msg/wind_sensors.hpp>
#include <functional>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "can_frame_parser.h"
#include "can_transceiver.h"
#include "cmn_hdrs/ros_info.h"

constexpr int QUEUE_SIZE        = 10;
constexpr int PLACEHOLDER_VALUE = 42;   // Placeholder value for debugging or testing
constexpr int PLACEHOLDER_MS    = 500;  // Timer to be replaced with callback in future

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
        // Subscribers
        // There is no timer because subscriber will respond to whatever data is published to the topic /Simulator
        sub_mock_ais_ships_ = this->create_subscription<custom_interfaces::msg::AISShips>(
          AIS_SHIPS_TOPIC, QUEUE_SIZE, std::bind(&CanSimIntf::mock_ais_callback, this, std::placeholders::_1));
        sub_mock_gps_ = this->create_subscription<custom_interfaces::msg::GPS>(
          MOCK_GPS_TOPIC, QUEUE_SIZE, std::bind(&CanSimIntf::gps_callback, this, std::placeholders::_1));
        sub_mock_wind_sensors_ = this->create_subscription<custom_interfaces::msg::WindSensors>(
          MOCK_WIND_SENSORS_TOPIC, QUEUE_SIZE, std::bind(&CanSimIntf::wind_callback, this, std::placeholders::_1));

        // Publishers
        pub_boat_sim_input_ =
          this->create_publisher<custom_interfaces::msg::CanSimToBoatSim>("boat_sim_input", QUEUE_SIZE);
        // Timer with 500ms delay
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(PLACEHOLDER_MS), std::bind(&CanSimIntf::timer_callback, this));
    }

private:
    // Subscriber
    void mock_ais_callback(const custom_interfaces::msg::AISShips::SharedPtr msg) const {}
    void gps_callback(const custom_interfaces::msg::GPS::SharedPtr msg) const {}
    void wind_callback(const custom_interfaces::msg::WindSensors::SharedPtr msg) const {}

    // Publisher
    void timer_callback()
    {
        // **CanSimToBoatSim**
        custom_interfaces::msg::CanSimToBoatSim can_sim_to_boat_sim;
        can_sim_to_boat_sim.sail_cmd.trim_tab_angle_degrees = PLACEHOLDER_VALUE;
        can_sim_to_boat_sim.heading.heading.heading         = PLACEHOLDER_VALUE;
        pub_boat_sim_input_->publish(can_sim_to_boat_sim);
    }

    // Subscriber Field Declarations
    rclcpp::Subscription<custom_interfaces::msg::AISShips>::SharedPtr    sub_mock_ais_ships_;
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr         sub_mock_gps_;
    rclcpp::Subscription<custom_interfaces::msg::WindSensors>::SharedPtr sub_mock_wind_sensors_;
    // Publisher Field Declarations
    rclcpp::Publisher<custom_interfaces::msg::CanSimToBoatSim>::SharedPtr pub_boat_sim_input_;
    // Timer object to allow our CamSimIntfFeedback node to perform action at x rate
    rclcpp::TimerBase::SharedPtr timer_;
};

class CanTrxRosIntf : public rclcpp::Node
{
public:
    CanTrxRosIntf()  //Our node which subs to topics
    : Node("CanTrxRosIntf")
    {
        // Subscribers
        // There is no timer because subscriber will respond to whatever data is published to the topic /Simulator
        sub_desired_heading_ = this->create_subscription<custom_interfaces::msg::DesiredHeading>(
          MOCK_GPS_TOPIC, QUEUE_SIZE, std::bind(&CanTrxRosIntf::desired_heading_callback, this, std::placeholders::_1));
        sub_sail_cmd_ = this->create_subscription<custom_interfaces::msg::SailCmd>(
          "sail_cmd", QUEUE_SIZE, std::bind(&CanTrxRosIntf::sail_cmd_callback, this, std::placeholders::_1));

        // Publishers
        pub_gps_ = this->create_publisher<custom_interfaces::msg::GPS>(GPS_TOPIC, QUEUE_SIZE);
        pub_filtered_wind_sensor_ =
          this->create_publisher<custom_interfaces::msg::WindSensor>(FILTERED_WIND_SENSOR_TOPIC, QUEUE_SIZE);
        pub_wind_sensors_ = this->create_publisher<custom_interfaces::msg::WindSensors>(WIND_SENSORS_TOPIC, QUEUE_SIZE);
        pub_batteries_    = this->create_publisher<custom_interfaces::msg::Batteries>(BATTERIES_TOPIC, QUEUE_SIZE);
        pub_data_sensors_ =
          this->create_publisher<custom_interfaces::msg::GenericSensors>(DATA_SENSORS_TOPIC, QUEUE_SIZE);
        pub_ais_ships_ = this->create_publisher<custom_interfaces::msg::AISShips>(AIS_SHIPS_TOPIC, QUEUE_SIZE);
        // Timer with 500ms delay
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(PLACEHOLDER_MS), std::bind(&CanTrxRosIntf::timer_callback, this));
    }

private:
    // Subscriber
    void desired_heading_callback(const custom_interfaces::msg::DesiredHeading::SharedPtr msg) const {}
    void sail_cmd_callback(const custom_interfaces::msg::SailCmd::SharedPtr msg) const {}

    // Publisher
    void timer_callback()
    {
        // ** GPS **
        custom_interfaces::msg::GPS gps_data;
        gps_data.heading.heading   = PLACEHOLDER_VALUE;
        gps_data.speed.speed       = PLACEHOLDER_VALUE;
        gps_data.lat_lon.latitude  = PLACEHOLDER_VALUE;
        gps_data.lat_lon.longitude = PLACEHOLDER_VALUE;
        // Publishes msg
        pub_gps_->publish(gps_data);

        // ** Filtered Wind Sensor **
        custom_interfaces::msg::WindSensor filtered_wind_sensor_data;
        filtered_wind_sensor_data.speed.speed = PLACEHOLDER_VALUE;
        filtered_wind_sensor_data.direction   = PLACEHOLDER_VALUE;
        // Publishes msg
        pub_filtered_wind_sensor_->publish(filtered_wind_sensor_data);

        // ** Wind Sensors **
        custom_interfaces::msg::WindSensors wind_sensors_data;
        for (auto & sensor : wind_sensors_data.wind_sensors) {
            sensor.direction   = PLACEHOLDER_VALUE;
            sensor.speed.speed = PLACEHOLDER_VALUE;
        }
        // Publishes msg
        pub_wind_sensors_->publish(wind_sensors_data);

        // ** Batteries **
        custom_interfaces::msg::Batteries batteries_data;
        for (auto & sensor : batteries_data.batteries) {
            sensor.voltage = PLACEHOLDER_VALUE;
            sensor.current = PLACEHOLDER_VALUE;
        }
        // Publishes msg
        pub_batteries_->publish(batteries_data);

        // ** Data Sensors **
        custom_interfaces::msg::HelperGenericSensor helper_gen_sensor_data;
        custom_interfaces::msg::GenericSensors      gen_sensor_data;
        helper_gen_sensor_data.id   = 0x0;  //uint8
        helper_gen_sensor_data.data = 0x0;  //uint64
        // Publishes msg
        gen_sensor_data.generic_sensors.push_back(helper_gen_sensor_data);
        pub_data_sensors_->publish(gen_sensor_data);

        // **AIS Ships**
        custom_interfaces::msg::HelperAISShip helper_ais_ship;
        custom_interfaces::msg::AISShips      ais_ships_data;
        helper_ais_ship.id                = PLACEHOLDER_VALUE;
        helper_ais_ship.cog.heading       = PLACEHOLDER_VALUE;
        helper_ais_ship.lat_lon.latitude  = PLACEHOLDER_VALUE;
        helper_ais_ship.lat_lon.longitude = PLACEHOLDER_VALUE;
        helper_ais_ship.length.dimension  = PLACEHOLDER_VALUE;
        helper_ais_ship.rot.rot           = PLACEHOLDER_VALUE;
        pub_ais_ships_->publish(ais_ships_data);
    }

    // Subscriber Field Declarations
    rclcpp::Subscription<custom_interfaces::msg::DesiredHeading>::SharedPtr sub_desired_heading_;
    rclcpp::Subscription<custom_interfaces::msg::SailCmd>::SharedPtr        sub_sail_cmd_;
    // Publisher Field Declarations
    rclcpp::Publisher<custom_interfaces::msg::GPS>::SharedPtr            pub_gps_;
    rclcpp::Publisher<custom_interfaces::msg::WindSensor>::SharedPtr     pub_filtered_wind_sensor_;
    rclcpp::Publisher<custom_interfaces::msg::WindSensors>::SharedPtr    pub_wind_sensors_;
    rclcpp::Publisher<custom_interfaces::msg::Batteries>::SharedPtr      pub_batteries_;
    rclcpp::Publisher<custom_interfaces::msg::GenericSensors>::SharedPtr pub_data_sensors_;
    rclcpp::Publisher<custom_interfaces::msg::AISShips>::SharedPtr       pub_ais_ships_;

    // Timer object to allow our CanTrxRosIntf node to perform action at x rate
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
    rclcpp::spin(std::make_shared<CanTrxRosIntf>());
    rclcpp::shutdown();
    return 0;
}
