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
#include "cmn_hdrs/shared_constants.h"

constexpr int QUEUE_SIZE = 10;
// constexpr int PLACEHOLDER_VALUE = 42;   // Placeholder value for debugging or testing
// constexpr int PLACEHOLDER_MS    = 500;  // Timer to be replaced with callback in future

namespace msg = custom_interfaces::msg;
using CAN::CanFrame;
using CAN::CanId;

static int mockCanFd()
{
    static std::mutex           mtx;
    std::lock_guard<std::mutex> lock(mtx);

    volatile static int fd = -1;
    if (fd == -1) {
        const static std::string tmp_file_template_str = "/tmp/CanSimIntfXXXXXX";
        std::vector<char>        tmp_file_template_cstr(
                 tmp_file_template_str.c_str(), tmp_file_template_str.c_str() + tmp_file_template_str.size() + 1);
        fd = mkstemp(tmp_file_template_cstr.data());
        if (fd == -1) {
            std::string err_msg = "Failed to open mock CAN fd with error: " + std::to_string(errno) + "(" +
                                  strerror(errno) + ")";  // NOLINT(concurrency-mt-unsafe)
            throw std::runtime_error(err_msg);
        }
        return fd;
    }
    return fd;
}

class CanTransceiverIntf : public rclcpp::Node
{
public:
    explicit CanTransceiverIntf() : Node("can_transceiver_node")
    {
        this->declare_parameter("enabled", true);

        if (!this->get_parameter("enabled").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "CAN Transceiver is DISABLED");
        } else {
            this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

            rclcpp::Parameter mode_param = this->get_parameter("mode");
            std::string       mode       = mode_param.as_string();

            if (mode == SYSTEM_MODE::PROD) {
                try {
                    can_trns_ = std::make_unique<CanTransceiver>();
                } catch (std::runtime_error err) {
                    RCLCPP_ERROR(this->get_logger(), "%s", err.what());
                    throw err;
                }
            } else if (mode == SYSTEM_MODE::DEV) {
                can_trns_ = std::make_unique<CanTransceiver>(mockCanFd());
            } else {
                std::string msg = "Error, invalid system mode" + mode;
                throw std::runtime_error(msg);
            }

            batteries_pub_ = this->create_publisher<msg::Batteries>(BATTERIES_TOPIC, QUEUE_SIZE);

            can_trns_->registerCanCbs({
              std::make_pair(
                CanId::BMS_P_DATA_FRAME_1,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
              std::make_pair(
                CanId::BMS_P_DATA_FRAME_2,
                std::function<void(const CanFrame &)>([this](const CanFrame & frame) { publishBattery(frame); })),
            });
        }
    }

private:
    std::unique_ptr<CanTransceiver> can_trns_;

    rclcpp::Publisher<msg::Batteries>::SharedPtr batteries_pub_;
    msg::Batteries                               batteries_;

    void publishBattery(const CanFrame & battery_frame)
    {
        CAN::Battery bat(battery_frame);

        size_t idx;
        for (size_t i = 0;; i++) {  // idx WILL be in range (can_frame_parser guarentees this)
            if (bat.id_ == CAN::Battery::BATTERY_IDS[i]) {
                idx = i;
                break;
            }
        }
        msg::HelperBattery & bat_msg = batteries_.batteries[idx];
        bat_msg                      = bat.toRosMsg();
        batteries_pub_->publish(batteries_);
    }
};

class CanSimIntf : public rclcpp::Node
{
public:
    explicit CanSimIntf() : Node("can_sim_node")
    {
        this->declare_parameter("enabled", true);

        if (!this->get_parameter("enabled").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "CAN Sim Intf is DISABLED");
        } else {
            this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

            rclcpp::Parameter mode_param = this->get_parameter("mode");
            std::string       mode       = mode_param.as_string();

            if (mode == SYSTEM_MODE::PROD) {
                RCLCPP_WARN(this->get_logger(), "CAN Sim Intf is not meant to run in production mode!");
            }

            int fd = mockCanFd();

            (void)fd;  // TODO(): flesh out Sim intf
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CanTransceiverIntf>());
    rclcpp::spin(std::make_shared<CanSimIntf>());
    rclcpp::shutdown();
    return 0;
}
