#include <chrono>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>

#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "local_transceiver.h"
#include "net_node.h"

/**
 * Local Transceiver Interface Node
 *
 */
class LocalTransceiverIntf : public NetNode
{
public:
    /**
     * @brief Construct a new Local Transceiver Intf Node
     *
     * @param lcl_trns Local Transceiver instance
     */
    explicit LocalTransceiverIntf(std::shared_ptr<LocalTransceiver> lcl_trns)
    : NetNode("local_transceiver_node"), lcl_trns_(lcl_trns)
    {
        static constexpr int  ROS_Q_SIZE     = 5;
        static constexpr auto TIMER_INTERVAL = std::chrono::milliseconds(500);
        pub_   = this->create_publisher<std_msgs::msg::String>(PLACEHOLDER_TOPIC_0_TOPIC, ROS_Q_SIZE);
        timer_ = this->create_wall_timer(TIMER_INTERVAL, std::bind(&LocalTransceiverIntf::pub_cb, this));
        sub_   = this->create_subscription<std_msgs::msg::String>(
          PLACEHOLDER_TOPIC_1_TOPIC, ROS_Q_SIZE, std::bind(&LocalTransceiverIntf::sub_cb, this, std::placeholders::_1));
    }

private:
    // Local Transceiver instance
    std::shared_ptr<LocalTransceiver> lcl_trns_;
    // Publishing timer
    rclcpp::TimerBase::SharedPtr timer_;
    // String is a placeholder pub and sub msg type - we will definitely define custom message types
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    // Placeholder subscriber object
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    /**
     * @brief Callback function to publish to onboard ROS network
     *
     */
    void pub_cb(/* placeholder */)
    {
        //TODO(jng468)
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network
     *
     */
    void sub_cb(std_msgs::msg::String /* placeholder */)
    {
        //TODO(jng468)
    }
};

int main(int argc, char * argv[])
{
    bool err = false;
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalTransceiver> lcl_trns = std::make_shared<LocalTransceiver>("PLACEHOLDER", SATELLITE_BAUD_RATE);
    try {
        std::shared_ptr<LocalTransceiverIntf> node = std::make_shared<LocalTransceiverIntf>(lcl_trns);
        try {
            rclcpp::spin(node);
        } catch (std::exception & e) {
            RCLCPP_ERROR(node->get_logger(), "%s", e.what());
            throw e;
        }
    } catch (std::exception & e) {
        std::cerr << e.what() << std::endl;
        err = true;
    }
    rclcpp::shutdown();
    return err ? -1 : 0;
}
