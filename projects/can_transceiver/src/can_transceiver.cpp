#include "can_transceiver.h"

#include <errno.h>
#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <stdexcept>
#include <thread>

#include "can_frame_parser.h"

using IFreq       = struct ifreq;
using SockAddr    = struct sockaddr;
using SockAddrCan = struct sockaddr_can;

using CAN_FP::CanFrame;
using CAN_FP::CanId;

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
    // See: https://www.kernel.org/doc/html/next/networking/can.html#how-to-use-socketcan
    static const char * CAN_INST = "can0";

    if ((sock_desc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        std::string err_msg = "Failed to open CAN socket with error: " + std::to_string(errno) + ": " +
                              strerror(errno);  // NOLINT(concurrency-mt-unsafe)
        throw std::runtime_error(err_msg);
    }

    IFreq       ifr;
    SockAddrCan addr;

    strncpy(ifr.ifr_name, CAN_INST, IFNAMSIZ);
    ioctl(sock_desc_, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_desc_, reinterpret_cast<const SockAddr *>(&addr), sizeof(addr)) < 0) {
        std::string err_msg = "Failed to bind CAN socket with error: " + std::to_string(errno) + ": " +
                              strerror(errno);  // NOLINT(concurrency-mt-unsafe)

        throw std::runtime_error(err_msg);
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
        // make sure the lock is acquired and released INSIDE the loop, otherwise send() will never get the lock
        std::lock_guard<std::mutex> lock(can_mtx_);
        CanFrame                    frame;
        ssize_t                     bytes_read = read(sock_desc_, &frame, sizeof(CanFrame));
        if (bytes_read > 0) {
            if (bytes_read != sizeof(CanFrame)) {
                std::cerr << "CAN read error: read " << bytes_read << "B but CAN frames are expected to be "
                          << sizeof(CanFrame) << "B" << std::endl;
            } else {
                onNewCanData(frame);
            }
        } else if (bytes_read < 0) {
            std::cerr << "CAN read error: " << errno << "(" << strerror(errno)  // NOLINT(concurrency-mt-unsafe)
                      << ")" << std::endl;
        }
    }
}

void CanTransceiver::send(const CanFrame & frame) const
{
    std::lock_guard<std::mutex> lock(can_mtx_);
    ssize_t                     bytes_written = write(sock_desc_, &frame, sizeof(CanFrame));
    if (bytes_written < 0) {
        std::cerr << "CAN write error: " << errno << "(" << strerror(errno)  // NOLINT(concurrency-mt-unsafe)
                  << ")" << std::endl;
    } else if (bytes_written != sizeof(CanFrame)) {
        std::cerr << "CAN write error: wrote " << bytes_written << "B but CAN frames are expected to be "
                  << sizeof(CanFrame) << "B" << std::endl;
    }
}

int mockCanFd(std::string tmp_file_template_str)
{
    std::vector<char> tmp_file_template_cstr(
      tmp_file_template_str.c_str(), tmp_file_template_str.c_str() + tmp_file_template_str.size() + 1);
    int fd = mkstemp(tmp_file_template_cstr.data());
    if (fd == -1) {
        std::string err_msg = "Failed to open mock CAN fd with error: " + std::to_string(errno) + "(" +
                              strerror(errno) + ")";  // NOLINT(concurrency-mt-unsafe)
        throw std::runtime_error(err_msg);
    }
    return fd;
}
