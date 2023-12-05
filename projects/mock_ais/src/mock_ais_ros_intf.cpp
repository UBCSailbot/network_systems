#include <chrono>
#include <custom_interfaces/msg/ais_ships.hpp>
#include <custom_interfaces/msg/gps.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription_base.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/wait_for_message.hpp>

#include "cmn_hdrs/ros_info.h"
#include "mock_ais.h"

class MockAisRosIntf : public rclcpp::Node
{
public:
    MockAisRosIntf() : Node("ais_transceiver_intf_node")
    {
        static constexpr int ROS_Q_SIZE = 5;
        this->declare_parameter("enabled", false);
        this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

        if (
          this->get_parameter("enabled").as_bool() &&
          this->get_parameter("mode").as_string() == "development") {  // TODO(hhenry01): replace with constant

            this->declare_parameter("publish_rate_ms", defaults::UPDATE_RATE_MS);
            this->declare_parameter("seed", defaults::SEED);
            this->declare_parameter("num_sim_ships", defaults::NUM_SIM_SHIPS);
            this->declare_parameter(
              "polaris_start_pos",
              std::vector<float>({defaults::POLARIS_START_POS[0], defaults::POLARIS_START_POS[1]}));

            rclcpp::Parameter publish_rate_ms_param   = this->get_parameter("publish_rate_ms");
            rclcpp::Parameter seed_param              = this->get_parameter("seed");
            rclcpp::Parameter num_sim_ships_param     = this->get_parameter("num_sim_ships");
            rclcpp::Parameter polaris_start_pos_param = this->get_parameter("polaris_start_pos");

            int64_t    publish_rate_ms   = publish_rate_ms_param.as_int();
            int64_t    seed              = seed_param.as_int();
            int64_t    num_sim_ships     = num_sim_ships_param.as_int();
            Vec2DFloat polaris_start_pos = {// annoyingly ugly type conversion :/
                                            static_cast<float>(polaris_start_pos_param.as_double_array()[0]),
                                            static_cast<float>(polaris_start_pos_param.as_double_array()[1])};

            mock_ais_ = std::make_unique<MockAis>(seed, num_sim_ships, polaris_start_pos);

            // The subscriber callback is very simple so it's just the following lambda function
            sub_ = this->create_subscription<custom_interfaces::msg::GPS>(
              MOCK_GPS_TOPIC, ROS_Q_SIZE, [&mock_ais_ = mock_ais_](custom_interfaces::msg::GPS mock_gps) {
                  mock_ais_->updatePolarisPos({mock_gps.lat_lon.latitude, mock_gps.lat_lon.longitude});
              });

            pub_   = this->create_publisher<custom_interfaces::msg::AISShips>(AIS_SHIPS_TOPIC, ROS_Q_SIZE);
            timer_ = this->create_wall_timer(
              std::chrono::milliseconds(publish_rate_ms), std::bind(&MockAisRosIntf::pubShipsCB, this));
        }
    }

private:
    std::unique_ptr<MockAis>                                       mock_ais_;
    rclcpp::TimerBase::SharedPtr                                   timer_;
    rclcpp::Publisher<custom_interfaces::msg::AISShips>::SharedPtr pub_;
    rclcpp::Subscription<custom_interfaces::msg::GPS>::SharedPtr   sub_;

    void pubShipsCB()
    {
        mock_ais_->tick();
        std::vector<AisShip>             ais_ships = mock_ais_->ships();
        custom_interfaces::msg::AISShips msg{};
        for (const AisShip & ais_ship : ais_ships) {
            custom_interfaces::msg::HelperAISShip helper_ship;
            helper_ship.set__id(static_cast<int32_t>(ais_ship.id_));
            custom_interfaces::msg::HelperHeading helper_head;
            helper_head.set__heading(ais_ship.heading_);
            helper_ship.set__cog(helper_head);
            custom_interfaces::msg::HelperSpeed helper_speed;
            helper_speed.set__speed(ais_ship.speed_);
            helper_ship.set__sog(helper_speed);
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
    // // TODO(): Parameterize mock + parameters vs deployment
    rclcpp::spin(std::make_shared<MockAisRosIntf>());
    rclcpp::shutdown();
    return 0;
}
