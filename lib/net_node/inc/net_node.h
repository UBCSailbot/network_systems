#include <boost/asio/io_context.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>

class NetNode : public rclcpp::Node
{
public:
    explicit NetNode(const std::string & node_name);
    static constexpr int BUFFER_SIZE = 1024;

private:
    boost::asio::io_context io_;
};
