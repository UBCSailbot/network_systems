#pragma once

#include <linux/can.h>

#include <string>

#include "rclcpp/rclcpp.hpp"

using IFreq       = struct ifreq;
using SockAddr    = struct sockaddr;
using SockAddrCan = struct sockaddr_can;

class CanTransceiver
{
public:
    virtual ~CanTransceiver() = 0;
    virtual void start()      = 0;

protected:
    virtual void receive() = 0;
};

class CanbusIntf : public CanTransceiver
{
public:
    explicit CanbusIntf(const std::string & can_inst);
    ~CanbusIntf();
    void start();

protected:
    std::thread receive_thread_;
    int         sock_desc_;
    void        receive();
    void        send(const can_frame & w_frame) const;
};

class CansimIntf : public CanbusIntf, public rclcpp::Node
{
    void receive() override;
};
