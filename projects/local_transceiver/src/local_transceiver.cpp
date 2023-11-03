#include "local_transceiver.h"

#include <boost/asio/read.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/write.hpp>
#include <boost/system/error_code.hpp>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>

#include "at_cmds.h"
#include "cmn_hdrs/ros_info.h"
#include "cmn_hdrs/shared_constants.h"
#include "custom_interfaces/msg/ais_ships.hpp"
#include "custom_interfaces/msg/gps.hpp"
#include "sensors.pb.h"

using Polaris::Sensors;
namespace bio = boost::asio;

LocalTransceiver::SensorBuf::SensorBuf(){};

void LocalTransceiver::SensorBuf::updateSensor(msg::GPS gps)
{
    std::lock_guard<std::mutex> lock(buf_mtx_);
    sensors_.mutable_gps()->set_heading(gps.heading.heading);
    sensors_.mutable_gps()->set_latitude(gps.lat_lon.latitude);
    sensors_.mutable_gps()->set_longitude(gps.lat_lon.longitude);
    sensors_.mutable_gps()->set_speed(gps.speed.speed);
}

void LocalTransceiver::SensorBuf::updateSensor(msg::AISShips ships)
{
    std::lock_guard<std::mutex> lock(buf_mtx_);
    sensors_.clear_ais_ships();
    for (const msg::HelperAISShip & ship : ships.ships) {
        Sensors::Ais * new_ship = sensors_.add_ais_ships();
        new_ship->set_id(ship.id);
        new_ship->set_cog(ship.cog.heading);
        new_ship->set_latitude(ship.lat_lon.latitude);
        new_ship->set_longitude(ship.lat_lon.longitude);
        new_ship->set_sog(ship.sog.speed);
        new_ship->set_rot(ship.rot.rot);
        new_ship->set_width(ship.width.dimension);
        new_ship->set_length(ship.length.dimension);
    }
}

Sensors LocalTransceiver::SensorBuf::sensors()
{
    std::lock_guard<std::mutex> lock(buf_mtx_);
    return sensors_;
}

LocalTransceiver::LocalTransceiver(const std::string & port_name, const uint32_t baud_rate) : serial_(io_, port_name)
{
    serial_.set_option(bio::serial_port_base::baud_rate(baud_rate));
};

LocalTransceiver::~LocalTransceiver()
{
    // Intentionally left blank
}

void LocalTransceiver::stop()
{
    std::lock_guard<std::mutex> lock(serial_mtx_);
    // TODO(Jng468): Flush the serial port
    serial_.close();  // Can throw an exception so cannot be put in the destructor
}

template <typename T>
void LocalTransceiver::onNewSensorData(T sensor)
{
    sensor_buf_.updateSensor(sensor);
}

bool LocalTransceiver::send()
{
    std::string data;
    // Make sure to get a copy of the sensors because repeated calls may give us different results
    Polaris::Sensors sensors = sensor_buf_.sensors();
    if (!sensors.SerializeToString(&data)) {
        std::cerr << "Failed to serialize sensors string" << std::endl;
        std::cerr << sensors.DebugString() << std::endl;
        return false;
    }
    if (data.size() >= MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES) {
        // if this proves to be a problem, we need a solution to split the data into multiple messages
        std::string err_string =
          "Data too large!\n"
          "Attempted: " +
          std::to_string(data.size()) + " bytes\n" + sensors.DebugString() +
          "\n"
          "No implementation to handle this!";
        throw std::length_error(err_string);
    }
    std::string msg = createOutMsg(data);

    std::lock_guard<std::mutex> lock(serial_mtx_);

    static constexpr int MAX_NUM_RETRIES = 20;
    for (int i = 0; i < MAX_NUM_RETRIES; i++) {
        // TODO(Jng468): Send the binary data
        send(msg);

        // Check SBD Session status to see if data was sent successfully
        send(AT::SBD_SESSION);
        std::string rsp_str = readLine();
        readLine();  // empty line after response
        if (checkOK()) {
            try {
                AT::SBDStatusResponse rsp(rsp_str);
                if (rsp.MOSuccess()) {
                    return true;
                }
            } catch (std::invalid_argument & e) {
                /* Catch response parsing exceptions */
            }
        }
    }
    return false;
}

std::string LocalTransceiver::debugSend(const std::string & cmd)
{
    send(cmd);
    // TODO(Jng468): Get the output and return it
    return "PLACEHOLDER";
}

std::string LocalTransceiver::receive()
{
    // TODO(Jng468)
    (void)serial_;  // Remove this line when implemented
    std::string msg = "placeholder";
    return msg;
}

void LocalTransceiver::send(const std::string & cmd) { bio::write(serial_, bio::buffer(cmd, cmd.size())); }

std::string LocalTransceiver::createOutMsg(const std::string & data)
{
    // TODO(Jng468)
    std::string msg = "Prepend command header + any others stuff" + data + " Append other stuff";
    return msg;
}

std::string LocalTransceiver::parseInMsg(const std::string & msg)
{
    // TODO(Jng468)
    (void)msg;
    // Separate data from payload header and other formatting
    std::string data = "placeholder";
    return data;
}

std::string LocalTransceiver::readLine()
{
    bio::streambuf buf;
    bio::read_until(serial_, buf, AT::DELIMITER);
    return std::string(
      bio::buffers_begin(buf.data()), bio::buffers_begin(buf.data()) + static_cast<int64_t>(buf.data().size()));
}

bool LocalTransceiver::checkOK()
{
    std::string status = readLine();
    return status == AT::STATUS_OK;
}
