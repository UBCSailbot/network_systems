#include "can_transceiver.h"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <cstring>
#include <stdexcept>
#include <thread>

#include "can_frame_parser.h"

using IFreq       = struct ifreq;
using SockAddr    = struct sockaddr;
using SockAddrCan = struct sockaddr_can;

using CAN::CanFrame;
using CAN::CanId;

void CanTransceiver::onNewCmd(const CanFrame & cmd_frame)
{
    // TODO(): IMPLEMENT
}

void CanTransceiver::onNewCanData(const CanFrame & frame) const
{
    CanId id{frame.can_id};
    if (read_callbacks_.contains(id)) {
        read_callbacks_.at(id)(frame);
    }
}

void CanTransceiver::registerCanCb(const std::pair<CanId, std::function<void(const CanFrame &)>> cb_kvp)
{
    auto [key, cb]       = cb_kvp;
    read_callbacks_[key] = cb;
}

void CanTransceiver::registerCanCbs(
  const std::initializer_list<std::pair<CanId, std::function<void(const CanFrame &)>>> & cb_kvps)
{
    for (const auto & cb_kvp : cb_kvps) {
        registerCanCb(cb_kvp);
    }
}

CanTransceiver::CanTransceiver()
{
    static const char * CAN_INST = "can0";

    if ((sock_desc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        throw std::runtime_error("Failed to open CAN socket");
    }

    IFreq       ifr;
    SockAddrCan addr;

    strncpy(ifr.ifr_name, CAN_INST, IFNAMSIZ);
    ioctl(sock_desc_, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_desc_, reinterpret_cast<const SockAddr *>(&addr), sizeof(addr)) < 0) {
        throw std::runtime_error("Failed to bind CAN socket");
    }

    receive_thread_ = std::thread(&CanTransceiver::receive, this);
}

CanTransceiver::CanTransceiver(int fd) : sock_desc_(fd)
{
    receive_thread_ = std::thread(&CanTransceiver::receive, this);
}

CanTransceiver::~CanTransceiver()
{
    close(sock_desc_);
    shutdown_flag_ = true;
    receive_thread_.join();
}

void CanTransceiver::receive()
{
    while (!shutdown_flag_) {
        CanFrame frame;
        size_t   bytes_read = 0;
        {  // scope the mutex
            std::lock_guard<std::mutex> lock(can_mtx_);
            bytes_read = read(sock_desc_, &frame, sizeof(CanFrame));
        }
        if (bytes_read > 0) {
            onNewCanData(frame);
        }
    }
}

void CanTransceiver::send(const CanFrame & frame) const
{
    std::lock_guard<std::mutex> lock(can_mtx_);
    if (write(sock_desc_, &frame, sizeof(can_frame)) != sizeof(can_frame)) {
        std::cerr << "Failed to write frame to CAN:" << std::endl;
    }
}
