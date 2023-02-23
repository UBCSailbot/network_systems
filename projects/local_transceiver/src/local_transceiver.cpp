#include "local_transceiver.h"

#include <mutex>

#include "boost/asio/serial_port.hpp"
#include "boost/asio/write.hpp"
#include "ros_info.h"
#include "sensors.pb.h"
#include "shared_constants.h"

LocalTransceiver::~LocalTransceiver(){};

void LocalTransceiver::onNewSensorData(/* placeholder */)
{
    static constexpr int MAX_NUM_RETRIES = 20;
    // process input ROS msg into byte string with protobuf
    std::string placeholder_byte_string = "deadbeef";
    for (int i = 0; i < MAX_NUM_RETRIES; i++) {
        if (send(placeholder_byte_string)) {
            break;
        }
    }
}

std::string LocalTransceiver::getRemoteData()
{
    std::string latest_data = receive();
    // format the data into the proper ROS msg format
    return "placeholder";
}

std::string LocalTransceiver::formatMsg(const std::string & data)
{
    std::string msg = "some formatting" + data;
    return msg;
}

std::string LocalTransceiver::parseMsg(const std::string & msg)
{
    (void)msg;
    // Separate data from payload header and other formatting
    std::string data = "placeholder";
    return data;
}

HwLocalTransceiver::HwLocalTransceiver(const std::string & serial_id) : serial_(io_) { serial_.open(serial_id); };

HwLocalTransceiver::~HwLocalTransceiver(){};

bool HwLocalTransceiver::send(const std::string & data)
{
    std::string msg = formatMsg(data);
    if (msg.size() >= MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES) {
        // format the message to be under the size limits
    }
    std::lock_guard<std::mutex> lock(serial_mtx_);
    boost::asio::write(serial_, boost::asio::buffer(msg, msg.size()));
    // if successful, else return false
    return true;
}

std::string HwLocalTransceiver::receive()
{
    // read from serial here
    std::string msg = "placeholder";
    return parseMsg(msg);
}

MockLocalTransceiver::MockLocalTransceiver() : Node("mock_local_transceiver_node")
{
    static constexpr int ROS_Q_SIZE = 5;
    pub_ = this->create_publisher<std_msgs::msg::String>(MOCK_LOCAL_TO_REMOTE_TRANSCEIVER_TOPIC, ROS_Q_SIZE);
    sub_ = this->create_subscription<std_msgs::msg::String>(
      MOCK_REMOTE_TO_LOCAL_TRANSCEIVER_TOPIC, ROS_Q_SIZE,
      std::bind(&MockLocalTransceiver::sub_callback, this, std::placeholders::_1));
}

MockLocalTransceiver::~MockLocalTransceiver() {}

bool MockLocalTransceiver::send(const std::string & data)
{
    auto msg = std_msgs::msg::String();
    msg.data = formatMsg(data);
    pub_->publish(msg);
    return true;
}

std::string MockLocalTransceiver::receive() { return parseMsg(latest_rcvd_msg_); }

void MockLocalTransceiver::sub_callback(std_msgs::msg::String::SharedPtr msg) { latest_rcvd_msg_ = msg->data; }
