// Include this module
#include "cached_fib.h"
// Include ROS headers
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int64.hpp>

namespace
{
constexpr int ROS_Q_SIZE    = 10;
constexpr int INIT_FIB_SIZE = 5;
}  // namespace

class CachedFibSubscriber : public rclcpp::Node
{
public:
    explicit CachedFibSubscriber(const std::size_t initSize) : Node("cached_fib_subscriber"), c_fib_(initSize)
    {
        this->declare_parameter("enabled", false);
        bool enabled = this->get_parameter("enabled").as_bool();
        if (enabled) {
            RCLCPP_INFO(this->get_logger(), "Running example cached fib node");
            sub_ = this->create_subscription<std_msgs::msg::UInt64>(
              CACHED_FIB_TOPIC, ROS_Q_SIZE,
              std::bind(&CachedFibSubscriber::topic_callback, this, std::placeholders::_1));
        }
    }

private:
    CachedFib                                              c_fib_;
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub_;
    void                                                   topic_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        int fibNum = this->c_fib_.getFib(msg->data);
        RCLCPP_INFO(this->get_logger(), "Fib num for '%lu' is '%d'", msg->data, fibNum);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CachedFibSubscriber>(INIT_FIB_SIZE));
    rclcpp::shutdown();
    return 0;
}
