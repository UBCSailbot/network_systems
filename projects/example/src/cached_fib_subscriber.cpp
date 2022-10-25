// Include this module
#include <functional>

#include "cached_fib.h"
// Include ROS headers
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"

namespace
{
constexpr int rosQSize = 10;
constexpr int initFibSize = 5;
}  // namespace

class CachedFibSubscriber : public rclcpp::Node
{
public:
    explicit CachedFibSubscriber(const std::size_t initSize) : Node("cached_fib_subscriber"), cFib(initSize)
    {
        sub = this->create_subscription<std_msgs::msg::UInt64>(
          CachedFibTopic, rosQSize, std::bind(&CachedFibSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    CachedFib cFib;
    void topic_callback(const std_msgs::msg::UInt64::SharedPtr msg)
    {
        int fibNum = this->cFib.getFib(msg->data);
        RCLCPP_INFO(this->get_logger(), "Fib num for '%d' is '%d'", msg->data, fibNum);
    }
    rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr sub;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CachedFibSubscriber>(initFibSize));
    rclcpp::shutdown();
    return 0;
}
