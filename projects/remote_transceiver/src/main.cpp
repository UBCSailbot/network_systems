#include <boost/asio/io_context.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "remote_transceiver.h"
#include "sailbot_db.h"

class RemoteTransceiverNode : public rclcpp::Node
{
public:
    RemoteTransceiverNode() : Node("remote_transceiver")
    {
        this->declare_parameter("db_name", "test");
        this->declare_parameter("host", remote_transceiver::TESTING_HOST);
        this->declare_parameter("port", static_cast<int>(remote_transceiver::TESTING_PORT));
        this->declare_parameter("num_threads", remote_transceiver::DEFAULT_NUM_IO_THREADS);

        rclcpp::Parameter db_name_param     = this->get_parameter("db_name");
        rclcpp::Parameter host_param        = this->get_parameter("host");
        rclcpp::Parameter port_param        = this->get_parameter("port");
        rclcpp::Parameter num_threads_param = this->get_parameter("num_threads");

        RCLCPP_INFO(
          this->get_logger(), "Running Remote Transceiver with database: %s, host: %s, port: %s, num_threads: %s",
          db_name_param.value_to_string().c_str(), host_param.value_to_string().c_str(),
          port_param.value_to_string().c_str(), num_threads_param.value_to_string().c_str());

        SailbotDB sailbot_db(db_name_param.as_string(), MONGODB_CONN_STR);
        if (!sailbot_db.testConnection()) {
            throw std::runtime_error("Failed to connect to database");
        }

        try {
            io_ = std::make_unique<bio::io_context>(num_threads_param.as_int());
            io_threads_.reserve(num_threads_param.as_int());
            bio::ip::address addr = bio::ip::make_address(host_param.as_string());
            const uint16_t   port = port_param.as_int();

            tcp::acceptor acceptor{*io_, {addr, port}};
            tcp::socket   socket{*io_};

            listener_ =
              std::make_unique<remote_transceiver::Listener>(*io_, std::move(acceptor), std::move(sailbot_db));
            listener_->run();

            for (std::thread & io_thread : io_threads_) {
                io_thread = std::thread([&io_ = io_]() { io_->run(); });
            }
        } catch (std::exception & e) {
            std::string msg = "Failed to run HTTP Server\n";
            msg += std::string(e.what());
            throw std::runtime_error(msg);
        }
    }

    ~RemoteTransceiverNode()
    {
        io_->stop();
        for (std::thread & io_thread : io_threads_) {
            io_thread.join();
        }
    }

private:
    std::unique_ptr<bio::io_context>              io_;
    std::vector<std::thread>                      io_threads_;
    std::unique_ptr<remote_transceiver::Listener> listener_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<RemoteTransceiverNode>());
    } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    rclcpp::shutdown();

    return 0;
}
