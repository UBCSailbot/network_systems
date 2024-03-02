#include "net_node.h"

#include <boost/asio/buffer.hpp>
#include <boost/asio/posix/stream_descriptor.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/streambuf.hpp>
#include <rclcpp/rclcpp.hpp>

namespace bio = boost::asio;

NetNode::NetNode(const std::string & node_name) : rclcpp::Node(node_name)
{
    bio::posix::stream_descriptor stdout_stream(io_, STDOUT_FILENO);
    bio::streambuf                stdout_buf;
    rclcpp::Logger                logger = this->get_logger();

    bio::async_read(
      io_, stdout_buf, [&logger, &stdout_buf](const boost::system::error_code & e, std::size_t bytes_transferred) {
          std::string output_str = std::string(
            bio::buffers_begin(stdout_buf.data()),
            bio::buffers_begin(stdout_buf.data()) + static_cast<int64_t>(stdout_buf.data().size()));
          RCLCPP_INFO(logger, "%s", output_str.c_str());
      });
}
