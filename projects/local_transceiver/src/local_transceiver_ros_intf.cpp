#include <chrono>
#include <custom_interfaces/msg/detail/batteries__struct.hpp>
#include <custom_interfaces/msg/detail/generic_sensors__struct.hpp>
#include <custom_interfaces/msg/detail/gps__struct.hpp>
#include <custom_interfaces/msg/detail/helper_ais_ship__struct.hpp>
#include <custom_interfaces/msg/detail/helper_battery__struct.hpp>
#include <custom_interfaces/msg/detail/l_path_data__struct.hpp>
#include <custom_interfaces/msg/detail/path__struct.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "local_transceiver.h"

/**
 * Local Transceiver Interface Node
 *
 */
class LocalTransceiverIntf : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Local Transceiver Intf Node
     *
     * @param lcl_trns Local Transceiver instance
     */
    explicit LocalTransceiverIntf(std::shared_ptr<LocalTransceiver> lcl_trns)
    : Node("local_transceiver_node"), lcl_trns_(lcl_trns)
    {
        static constexpr int  ROS_Q_SIZE     = 5;
        static constexpr auto TIMER_INTERVAL = std::chrono::milliseconds(500);
        pub_   = this->create_publisher<std_msgs::msg::String>(PLACEHOLDER_TOPIC_0_TOPIC, ROS_Q_SIZE);
        timer_ = this->create_wall_timer(TIMER_INTERVAL, std::bind(&LocalTransceiverIntf::pub_cb, this));

        // subscriber nodes
        sub_wind_sensor = this->create_subscription<custom_interfaces::msg::WindSensors>(
          WIND_SENSORS_TOPIC, ROS_Q_SIZE,
          std::bind(&LocalTransceiverIntf::sub_wind_sensor_cb, this, std::placeholders::_1));
        sub_batteries = this->create_subscription<custom_interfaces::msg::HelperBattery>(
          BATTERIES_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_batteries_cb, this, std::placeholders::_1));
        sub_data_sensors = this->create_subscription<custom_interfaces::msg::GenericSensors>(
          DATA_SENSORS_TOPIC, ROS_Q_SIZE,
          std::bind(&LocalTransceiverIntf::sub_data_sensors_cb, this, std::placeholders::_1));
        sub_ais_ships = this->create_subscription<custom_interfaces::msg::AISShips>(
          AIS_SHIPS_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_ais_ships_cb, this, std::placeholders::_1));
        sub_gps = this->create_subscription<custom_interfaces::msg::GPS>(
          GPS_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_gps_cb, this, std::placeholders::_1));

        // TODO(Jng468): what is the topic for local path data? :(
        // sub_local_path_data = this->create_subscription<custom_interfaces::msg::LPathData>(TOPIC);
    }

private:
    // Local Transceiver instance
    std::shared_ptr<LocalTransceiver> lcl_trns_;
    // Publishing timer
    rclcpp::TimerBase::SharedPtr timer_;
    // String is a placeholder pub and sub msg type - we will definitely define custom message types
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    // Placeholder subscriber object

    rclcpp::Subscription<custom_interfaces::msg::WindSensors>::SharedPtr    sub_wind_sensor;
    rclcpp::Subscription<custom_interfaces::msg::HelperBattery>::SharedPtr  sub_batteries;
    rclcpp::Subscription<custom_interfaces::msg::GenericSensors>::SharedPtr sub_data_sensors;
    rclcpp::Subscription<custom_interfaces::msg::AISShips>::SharedPtr       sub_ais_ships;
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr            sub_gps;
    //rclcpp::Subscription<custom_interfaces::msg::LPathData>::SharedPtr    sub_local_path_data;

    /**
     * @brief Callback function to publish to onboard ROS network
     *
     */
    void pub_cb(/* placeholder */)
    {
        // TODO(jng468)
        // https://github.com/UBCSailbot/network_systems/blob/user/hhenry01/87-can-frame-parsing/projects/example/src/cached_fib_ros_intf.cpp
        std::string recent_data = lcl_trns_->receive();  //receives most recent data from remote server
        auto        msg         = std_msgs::msg::String();
        msg.data                = recent_data;
        pub_->publish(msg);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for wind sensors
     */
    void sub_wind_sensor_cb(custom_interfaces::msg::WindSensors in_msg)
    {
        custom_interfaces::msg::WindSensors data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for batteries
     */
    void sub_batteries_cb(custom_interfaces::msg::HelperBattery in_msg)
    {
        std::vector<custom_interfaces::msg::HelperBattery> batteries;  // vector to store batteries from ROS
        custom_interfaces::msg::Batteries                  send_data;  // Batteries type to update sensor

        batteries.push_back(in_msg);
        if (batteries.size() >= 2) {
            std::array<custom_interfaces::msg::HelperBattery, 2> batteries_array;  // array holder
            copy(batteries.begin(), batteries.end(), batteries_array.begin());     // copy vector batteries into array
            send_data.set__batteries(batteries_array);

            lcl_trns_->updateSensor(send_data);
            batteries.clear();                        // clear vector
            for (auto & content : batteries_array) {  // clear array
                std::destroy_at(&content);
            }
        }
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for generic sensors
     */
    void sub_data_sensors_cb(custom_interfaces::msg::GenericSensors in_msg)
    {
        custom_interfaces::msg::GenericSensors data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for ais ships
     */
    void sub_ais_ships_cb(custom_interfaces::msg::AISShips in_msg)
    {
        custom_interfaces::msg::AISShips data = in_msg;
        lcl_trns_->updateSensor(data);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network for GPS
     */
    void sub_gps_cb(custom_interfaces::msg::GPS in_msg)
    {
        custom_interfaces::msg::GPS data = in_msg;
        lcl_trns_->updateSensor(data);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalTransceiver> lcl_trns = std::make_shared<LocalTransceiver>("PLACEHOLDER", SATELLITE_BAUD_RATE);
    rclcpp::spin(std::make_shared<LocalTransceiverIntf>(lcl_trns));
    rclcpp::shutdown();
    return 0;
}
