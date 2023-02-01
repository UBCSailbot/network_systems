#include "can_transceiver.h"

#include <linux/can.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <stdexcept>
#include <thread>

#include "can_frame_parser.h"

namespace
{
std::string get_can_frame_dbg_str(const can_frame & frame)
{
    std::string str = "ID: " + std::to_string(frame.can_id) + "\nData:";
    // Bytes ascending from left to right matches candump output order
    for (unsigned int i = 0; i < CAN_MAX_DLEN; i++) {
        str += " " + std::to_string(frame.data[i]);
    }
    return str;
}
}  // namespace

CanTransceiver::~CanTransceiver(){};

CanbusIntf::CanbusIntf(const std::string & can_inst)
{
    if ((sock_desc_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        throw std::runtime_error("Failed to open CAN socket");
    }

    IFreq       ifr;
    SockAddrCan addr;

    strncpy(ifr.ifr_name, can_inst.c_str(), IFNAMSIZ);
    ioctl(sock_desc_, SIOCGIFINDEX, &ifr);

    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_desc_, reinterpret_cast<const SockAddr *>(&addr), sizeof(addr)) < 0) {
        throw std::runtime_error("Failed to bind CAN socket");
    }
}

CanbusIntf::~CanbusIntf() { close(sock_desc_); }

void CanbusIntf::start() { receive_thread_ = std::thread(&CanbusIntf::receive, this); }

void CanbusIntf::receive()
{
    can_frame r_frame;
    while (true) {
        read(sock_desc_, &r_frame, sizeof(can_frame));
    }
}

void CanbusIntf::send(const can_frame & w_frame) const
{
    if (write(sock_desc_, &w_frame, sizeof(can_frame)) != sizeof(can_frame)) {
        // Log error
        std::cout << "Failed to write frame to CAN:" << std::endl;
        std::cout << get_can_frame_dbg_str(w_frame) << std::endl;
    }
}

void CansimIntf::receive() { CanbusIntf::receive(); }
