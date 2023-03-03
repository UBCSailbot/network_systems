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

namespace
{

/**
 * @brief Format a CAN frame into a string for debugging and logging
 * 
 * @param frame CAN frame to format
 * @return String representation of frame
 */
std::string fmtCanFrameDbgStr(const CanFrame & frame)
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

void CanTransceiver::onNewCmd(CanId id /*, other data fields...*/)
{
    switch (id) {
        case RudderCmd: {
            struct RudderCmd cmd;
            CanFrame         frame = cmd.toLinuxCan();
            send(frame);
            break;
        }
        default:
            // log error
            std::cerr << "Unknown CAN ID: " << id << std::endl;
    }
}

void CanTransceiver::onNewCanData(const CanFrame & frame)
{
    switch (frame.can_id) {
        case Placeholder0: {
            struct Placeholder0 inst(frame);
            // buffer data (PLACEHOLDER METHOD)
            memcpy(sensor_buf_, &inst, sizeof(inst));
            break;
        }
        case Placeholder1: {
            struct Placeholder1 inst(frame);
            // buffer data (PLACEHOLDER METHOD)
            memcpy(sensor_buf_, &inst, sizeof(inst));
            break;
        }
        default:
            // log error
            std::cerr << "Unknown CAN ID: " << frame.can_id << std::endl;
    }
}

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

    receive_thread_ = std::thread(&CanbusIntf::receive, this);
}

CanbusIntf::~CanbusIntf() { close(sock_desc_); }

void CanbusIntf::receive()
{
    CanFrame frame;
    while (true) {
        read(sock_desc_, &frame, sizeof(can_frame));
        onNewCanData(frame);
    }
}

void CanbusIntf::send(const CanFrame & frame) const
{
    if (write(sock_desc_, &frame, sizeof(can_frame)) != sizeof(can_frame)) {
        // Log error
        std::cerr << "Failed to write frame to CAN:" << std::endl;
        std::cerr << fmtCanFrameDbgStr(frame) << std::endl;
    }
}

//===========================================================================================
// Simulation Interface Components
//===========================================================================================

//void CanSimIntf::receive() {}
//void CanSimIntf::sendData() {}
