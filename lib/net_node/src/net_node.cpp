#include "net_node.h"

#include <rclcpp/rclcpp.hpp>

NetNode::NetNode(const std::string & node_name)
: rclcpp::Node(node_name), stdout_stream_(STDOUT_FILENO, this->get_logger())
{
    std::cout.rdbuf(stdout_stream_.rdbuf());
    stdout_stream_ << "test" << std::endl;
}

NetNode::~NetNode() {}

LogOverride::LogOverride(int fd, rclcpp::Logger logger) : fd_(fd), logger_(logger) {}

LogOverride & LogOverride::operator<<(const std::string & str)
{
    std::cerr << "HERE" << std::endl;
    switch (fd_) {
        case STDIN_FILENO:
            RCLCPP_INFO(logger_, "%s", str.c_str());
            break;
        case STDOUT_FILENO:
            RCLCPP_ERROR(logger_, "%s", str.c_str());
            break;
    }
    return *this;
}
