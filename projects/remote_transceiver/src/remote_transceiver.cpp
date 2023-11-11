#include "remote_transceiver.h"

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/core/error.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/string_type.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/core/ignore_unused.hpp>
#include <boost/system/error_code.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db.h"
#include "sensors.pb.h"

using remote_transceiver::HTTPServer;
namespace http_client = remote_transceiver::http_client;

remote_transceiver::MOMsgParams::MOMsgParams(const std::string & query_string)
{
    static const std::string DATA_KEY = "&data=";

    size_t      data_key_idx  = query_string.find(DATA_KEY);
    std::string iridium_mdata = query_string.substr(0, data_key_idx);
    params_.data_             = query_string.substr(data_key_idx + DATA_KEY.size(), query_string.size());

    std::vector<std::string> split_strings;
    boost::algorithm::split(split_strings, iridium_mdata, boost::is_any_of("?=&"));

    constexpr uint8_t IMEI_IDX   = 1;
    constexpr uint8_t SERIAL_IDX = 3;
    constexpr uint8_t MOMSN_IDX  = 5;
    constexpr uint8_t TIME_IDX   = 7;
    constexpr uint8_t LAT_IDX    = 9;
    constexpr uint8_t LON_IDX    = 11;
    constexpr uint8_t CEP_IDX    = 13;

    params_.imei_          = std::stoi(split_strings[IMEI_IDX]);
    params_.serial_        = std::stoi(split_strings[SERIAL_IDX]);
    params_.momsn_         = std::stoi(split_strings[MOMSN_IDX]);
    params_.transmit_time_ = split_strings[TIME_IDX];
    params_.lat_           = std::stof(split_strings[LAT_IDX]);
    params_.lon_           = std::stof(split_strings[LON_IDX]);
    params_.cep_           = std::stoi(split_strings[CEP_IDX]);
}
HTTPServer::HTTPServer(tcp::socket socket, SailbotDB db) : socket_(std::move(socket)), db_(std::move(db)) {}

void HTTPServer::run() { readReq(); }

void HTTPServer::runServer(tcp::acceptor & acceptor, tcp::socket & socket, SailbotDB & db)
{
    acceptor.async_accept(socket, [&](beast::error_code e) {
        if (!e) {
            std::make_shared<HTTPServer>(std::move(socket), std::move(db))->run();
        }
        runServer(acceptor, socket, db);
    });
}

void HTTPServer::readReq()
{
    std::shared_ptr<HTTPServer> self = shared_from_this();
    http::async_read(socket_, buf_, req_, [self](beast::error_code e, std::size_t /*bytesTransferred*/) {
        if (!e) {
            self->processReq();
        } else {
            std::cerr << "Error: " << e.value() << " " << e.message() << std::endl;
            std::cerr << self->req_ << std::endl;
        }
    });
}

void HTTPServer::processReq()
{
    res_.version(req_.version());
    res_.keep_alive(false);  // Expect very infrequent requests, so disable keep alive

    switch (req_.method()) {
        case http::verb::post:
            doPost();
            break;
        case http::verb::get:
            doGet();
            break;
        default:
            doBadReq();
    }
    writeRes();
}

void HTTPServer::doBadReq()
{
    res_.result(http::status::bad_request);
    res_.set(http::field::content_type, "text/plain");
    beast::ostream(res_.body()) << "Invalid request method: " << req_.method_string();
}

// https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
// IMPORTANT: Have 3 seconds to send HTTP status 200, so do not process data on same thread before responding
void HTTPServer::doPost()
{
    beast::string_view content_type = req_["content-type"];
    if (content_type == "application/x-www-form-urlencoded") {
        res_.result(http::status::ok);
        std::shared_ptr<HTTPServer> self = shared_from_this();
        std::thread                 post_thread([self]() {
            std::string         query_string = beast::buffers_to_string(self->req_.body().data());
            MOMsgParams::Params params       = MOMsgParams(query_string).params_;
            if (!params.data_.empty()) {
                Polaris::Sensors       sensors;
                SailbotDB::RcvdMsgInfo info = {params.lat_, params.lon_, params.cep_, params.transmit_time_};
                sensors.ParseFromString(params.data_);
                if (!self->db_.storeNewSensors(sensors, info)) {
                    std::cerr << "Error, failed to store data received from:\n" << info << std::endl;
                };
            }
        });
        post_thread.detach();
    } else {
        res_.result(http::status::unsupported_media_type);
        res_.set(http::field::content_type, "text/plain");
        beast::ostream(res_.body()) << "Server does not support POST requests of type: " << content_type;
    }
}

void HTTPServer::doGet()
{
    res_.result(http::status::ok);
    res_.set(http::field::server, "Sailbot Remote Transceiver");
    res_.set(http::field::content_type, "text/plain");
    beast::ostream(res_.body()) << "PLACEHOLDER\r\n";
}

void HTTPServer::writeRes()
{
    res_.set(http::field::content_length, std::to_string(res_.body().size()));

    std::shared_ptr<HTTPServer> self = shared_from_this();
    http::async_write(socket_, res_, [self](beast::error_code e, std::size_t /*bytesWritten*/) {
        self->socket_.shutdown(tcp::socket::shutdown_send, e);
    });
}

std::pair<http::status, std::string> http_client::get(ConnectionInfo info)
{
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

    auto [host, port, target] = info.get();

    tcp::resolver::results_type const results = resolver.resolve(host, port);
    bio::connect(socket, results.begin(), results.end());

    http::request<http::string_body> req{http::verb::get, target, HTTP_VERSION};
    req.set(http::field::host, host);
    req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    http::write(socket, req);

    beast::flat_buffer buf;

    http::response<http::dynamic_body> res;
    http::read(socket, buf, res);

    boost::system::error_code e;
    socket.shutdown(tcp::socket::shutdown_both, e);

    http::status status = res.base().result();
    if (status == http::status::ok) {
        std::string result = beast::buffers_to_string(res.body().data());
        return {status, result};
    }
    return {status, ""};
}

http::status http_client::post(ConnectionInfo info, std::string content_type, const std::string & body)
{
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

    auto [host, port, target] = info.get();

    tcp::resolver::results_type const results = resolver.resolve(host, port);
    bio::connect(socket, results.begin(), results.end());

    http::request<http::string_body> req{http::verb::post, target, HTTP_VERSION};
    req.set(http::field::host, host);
    req.set(http::field::user_agent, BOOST_BEAST_VERSION_STRING);
    req.set(http::field::content_type, content_type);
    req.set(http::field::content_length, std::to_string(body.size()));
    req.body() = body;

    req.prepare_payload();
    http::write(socket, req);

    beast::flat_buffer buf;

    http::response<http::dynamic_body> res;
    http::read(socket, buf, res);

    boost::system::error_code e;
    socket.shutdown(tcp::socket::shutdown_both, e);

    http::status status = res.base().result();
    return status;
}
