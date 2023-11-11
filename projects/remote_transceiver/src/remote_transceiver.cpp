#include "remote_transceiver.h"

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
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
            std::string query_string = beast::buffers_to_string(self->req_.body().data());
            MOMsgParams params       = MOMsgParams(query_string);
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

std::pair<http::status, std::string> http_client::get(
  const std::string & host, const std::string & port, const std::string & target)
{
    bio::io_context io;
    tcp::socket     socket{io};
    tcp::resolver   resolver{io};

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
