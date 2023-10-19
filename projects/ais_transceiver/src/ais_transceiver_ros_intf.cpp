#include <chrono>
#include <memory>

#include "ais_ship.h"
#include "custom_interfaces/msg/ais_ships.hpp"
#include "custom_interfaces/msg/detail/ais_ships__struct.hpp"
#include "custom_interfaces/msg/detail/helper_ais_ship__struct.hpp"
#include "custom_interfaces/msg/detail/helper_heading__struct.hpp"
#include "custom_interfaces/msg/detail/helper_lat_lon__struct.hpp"
#include "custom_interfaces/msg/detail/helper_speed__struct.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "mock_ais_sim.h"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription_base.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "ros_info.h"

class AisTransceiverIntf : public rclcpp::Node
{
public:
    AisTransceiverIntf(std::shared_ptr<AisTransceiver> ais_trns_, std::chrono::milliseconds publish_rate)
    : Node("ais_transceiver_intf_node"), ais_trns_(ais_trns_)
    {
        static constexpr int ROS_Q_SIZE = 5;
        pub_ = this->create_publisher<custom_interfaces::msg::AISShips>(AIS_SHIPS_TOPIC, ROS_Q_SIZE);
        sub_ = this->create_subscription<custom_interfaces::msg::GPS>(
          MOCK_GPS_TOPIC, ROS_Q_SIZE, [&ais_trns_](custom_interfaces::msg::GPS mock_gps) {
              ais_trns_->updatePolarisPos({mock_gps.lat_lon.latitude, mock_gps.lat_lon.longitude});
          });
        timer_ = this->create_wall_timer(publish_rate, std::bind(&AisTransceiverIntf::pub_cb, this));
    }

private:
    std::shared_ptr<AisTransceiver>                                ais_trns_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
    rclcpp::Publisher<custom_interfaces::msg::AISShips>::SharedPtr pub_;
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr   sub_;

    void pub_cb()
    {
        std::vector<AisShip>             ais_ships = ais_trns_->ships();
        custom_interfaces::msg::AISShips msg       = custom_interfaces::msg::AISShips();
        for (const AisShip & ais_ship : ais_ships) {
            custom_interfaces::msg::HelperAISShip helper_ship;
            helper_ship.set__id(static_cast<int32_t>(ais_ship.id_));
            custom_interfaces::msg::HelperHeading helper_head;
            helper_head.set__heading(ais_ship.heading_);
            helper_ship.set__heading(helper_head);
            custom_interfaces::msg::HelperSpeed helper_speed;
            helper_speed.set__speed(ais_ship.speed_);
            helper_ship.set__speed(helper_speed);
            custom_interfaces::msg::HelperLatLon lat_lon;
            lat_lon.set__latitude(ais_ship.lat_lon_[0]);
            lat_lon.set__longitude(ais_ship.lat_lon_[1]);
            helper_ship.set__lat_lon(lat_lon);

            msg.ships.push_back(helper_ship);
        }
        pub_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    // AisTransceiver transceiver;
    // // TODO(): Parameterize mock + parameters vs deployment
    // uint32_t                  seed             = 12345;
    // uint32_t                  num_ships        = 20;
    // std::chrono::milliseconds sim_tick_rate_ms = std::chrono::milliseconds(500);  // Boat simulator also runs at 2Hz
    // transceiver                                = MockAisSim(seed, num_ships, sim_tick_rate_ms, );
    // rclcpp::spin(std::make_shared<AisTransceiverIntf>());
    return 0;
}
