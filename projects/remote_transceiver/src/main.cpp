#include <iostream>
#include <memory>
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

        rclcpp::Parameter db_name_param = this->get_parameter("db_name");
        rclcpp::Parameter host_param    = this->get_parameter("host");
        rclcpp::Parameter port_param    = this->get_parameter("port");

        RCLCPP_INFO(
          this->get_logger(), "Running Remote Transceiver with database: %s, host: %s, port: %s",
          db_name_param.value_to_string().c_str(), host_param.value_to_string().c_str(),
          port_param.value_to_string().c_str());

        SailbotDB sailbot_db(db_name_param.as_string(), MONGODB_CONN_STR);
        if (!sailbot_db.testConnection()) {
            throw std::runtime_error("Failed to connect to database");
        }

        try {
            bio::ip::address addr = bio::ip::make_address(host_param.as_string());
            const uint16_t   port = port_param.as_int();

            tcp::acceptor acceptor{io_, {addr, port}};
            tcp::socket   socket{io_};

            listener_ = std::make_shared<remote_transceiver::Listener>(io_, std::move(acceptor), std::move(sailbot_db));
            listener_->run();

            io_thread_ = std::thread([&io_ = io_]() { io_.run(); });
        } catch (std::exception & e) {
            std::string msg = "Failed to run HTTP Server\n";
            msg += std::string(e.what());
            throw std::runtime_error(msg);
        }
    }

    ~RemoteTransceiverNode()
    {
        io_.stop();
        io_thread_.join();
    }

private:
    bio::io_context                               io_;
    std::thread                                   io_thread_;
    std::shared_ptr<remote_transceiver::Listener> listener_;
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
