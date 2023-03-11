#include <chrono>
#include <memory>

#include "local_transceiver.h"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "ros_info.h"
#include "std_msgs/msg/string.hpp"

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
    : Node("local_transceiver_intf_node"), lcl_trns_(lcl_trns)
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;

    /**
     * @brief Callback function to publish to onboard ROS network
     * 
     */
    void pub_cb(/* placeholder */)
    {
        std::string some_msg_content = lcl_trns_->getRemoteData();
        auto        msg              = std_msgs::msg::String();
        msg.data                     = some_msg_content;
        pub_->publish(msg);
    }

    /**
     * @brief Callback function to subscribe to the onboard ROS network
     * 
     */
    void sub_cb(std_msgs::msg::String /* placeholder */) { lcl_trns_->onNewSensorData(); }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<LocalTransceiver> lcl_trns;
    constexpr bool                    IS_HW_RUN = true;  // placeholder
    if (IS_HW_RUN) {
        std::shared_ptr<HwLocalTransceiver> hw_lcl_trns = std::make_shared<HwLocalTransceiver>("PLACEHOLDER");
        lcl_trns                                        = hw_lcl_trns;
    } else {
        std::shared_ptr<MockLocalTransceiver> mck_lcl_trns = std::make_shared<MockLocalTransceiver>();
        lcl_trns                                           = mck_lcl_trns;
    }
    rclcpp::spin(std::make_shared<LocalTransceiverIntf>(lcl_trns));
    rclcpp::shutdown();
    return 0;
}
