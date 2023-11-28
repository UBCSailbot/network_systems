#include <boost/asio/io_context.hpp>
#include <iostream>
#include <memory>
#include <rclcpp/parameter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>

#include "cmn_hdrs/shared_constants.h"
#include "remote_transceiver.h"
#include "sailbot_db.h"

class RemoteTransceiverRosIntf : public rclcpp::Node
{
public:
    RemoteTransceiverRosIntf() : Node("remote_transceiver")
    {
        this->declare_parameter("mode", rclcpp::PARAMETER_STRING);

        rclcpp::Parameter mode_param = this->get_parameter("mode");
        std::string       mode       = mode_param.as_string();

        std::string default_db_name;
        std::string default_host;
        int64_t     default_port;
        int64_t     default_num_threads = remote_transceiver::DEFAULT_NUM_IO_THREADS;

        if (mode == SYSTEM_MODE::PROD) {
            default_db_name = remote_transceiver::PROD_DB_NAME;
            default_host    = remote_transceiver::PROD_HOST;
            default_port    = remote_transceiver::PROD_PORT;
        } else if (mode == SYSTEM_MODE::DEV) {
            default_db_name = "test";
            default_host    = remote_transceiver::TESTING_HOST;
            default_port    = remote_transceiver::TESTING_PORT;

        } else {
            std::string msg = "Error, invalid system mode" + mode;
            throw std::runtime_error(msg);
        }

        this->declare_parameter("db_name", default_db_name);
        this->declare_parameter("host", default_host);
        this->declare_parameter("port", default_port);
        this->declare_parameter("num_threads", default_num_threads);

        rclcpp::Parameter db_name_param     = this->get_parameter("db_name");
        rclcpp::Parameter host_param        = this->get_parameter("host");
        rclcpp::Parameter port_param        = this->get_parameter("port");
        rclcpp::Parameter num_threads_param = this->get_parameter("num_threads");

        std::string db_name     = db_name_param.as_string();
        std::string host        = host_param.as_string();
        int64_t     port        = port_param.as_int();
        int64_t     num_threads = num_threads_param.as_int();

        RCLCPP_INFO(
          this->get_logger(),
          "Running Remote Transceiver in mode: %s, with database: %s, host: %s, port: %s, num_threads: %s",
          mode.c_str(), db_name.c_str(), host.c_str(), std::to_string(port).c_str(),
          std::to_string(num_threads).c_str());

        SailbotDB sailbot_db(db_name, MONGODB_CONN_STR);
        if (!sailbot_db.testConnection()) {
            throw std::runtime_error("Failed to connect to database");
        }

        try {
            io_ = std::make_unique<bio::io_context>(num_threads);
            io_threads_.reserve(num_threads);
            bio::ip::address addr = bio::ip::make_address(host);

            tcp::acceptor acceptor{*io_, {addr, static_cast<uint16_t>(port)}};
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

    ~RemoteTransceiverRosIntf()
    {
        io_->stop();
        for (std::thread & io_thread : io_threads_) {
            io_thread.join();
        }
    }

private:
    std::unique_ptr<remote_transceiver::Listener> listener_;
    std::unique_ptr<bio::io_context>              io_;
    std::vector<std::thread>                      io_threads_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<RemoteTransceiverRosIntf>());
    } catch (std::runtime_error & e) {
        std::cerr << e.what() << std::endl;
        return -1;
    }
    rclcpp::shutdown();

    return 0;
}
