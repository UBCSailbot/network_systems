//https://stackoverflow.com/a/55489194
#include <rclcpp/rclcpp.hpp>

class LogOverride : public std::ostream
{
public:
    LogOverride(int fd, rclcpp::Logger logger);
    LogOverride & operator<<(const std::string & str);

private:
    const int            fd_;
    const rclcpp::Logger logger_;
};
class NetNode : public rclcpp::Node
{
public:
    explicit NetNode(const std::string & node_name);
    ~NetNode();

private:
    LogOverride stdout_stream_;
};
