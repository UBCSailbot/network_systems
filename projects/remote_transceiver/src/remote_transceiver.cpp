#include "remote_transceiver.h"

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/core/error.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/string_type.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/http/field.hpp>
#include <boost/beast/http/write.hpp>
#include <boost/core/ignore_unused.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db.h"

namespace beast = boost::beast;
namespace http  = beast::http;
namespace bio   = boost::asio;
using tcp       = boost::asio::ip::tcp;

class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    explicit HTTPServer(tcp::socket socket) : socket_(std::move(socket)) {}
    void run() { readReq(); }

private:
    struct MOMsgParams
    {
        uint64_t imei_;
        uint32_t serial_;  // Don't know the max size
        uint16_t momsn_;
        // Date and time. Ex: "21-10-31 10:41:50" - make Date and Time objects if we end up needing this.
        std::string transmit_time_;
        float       lat_;
        float       lon_;
        uint32_t    cep_;   // estimate of the accuracy (in km) of the reported lat_ lon_ fields
        std::string data_;  // hex-encoded

        /**
         * @brief Construct a new MOMsg object
         *
         * @param query_string
         */
        // Example: ?imei=1234&serial=5678&momsn=9123&transmit_time=21-10-31 10:41:50&iridium_latitude=12.34&iridium_longitude=56.78&iridium_cep=2&data=A1B2C3
        explicit MOMsgParams(const std::string & query_string)
        {
            std::vector<std::string> split_strings;
            boost::split(split_strings, query_string, boost::is_any_of("?=&"));

            constexpr uint8_t IMEI_IDX   = 1;
            constexpr uint8_t SERIAL_IDX = 3;
            constexpr uint8_t MOMSN_IDX  = 5;
            constexpr uint8_t TIME_IDX   = 7;
            constexpr uint8_t LAT_IDX    = 9;
            constexpr uint8_t LON_IDX    = 11;
            constexpr uint8_t CEP_IDX    = 13;
            constexpr uint8_t DATA_IDX   = 15;

            imei_          = std::stoi(split_strings[IMEI_IDX]);
            serial_        = std::stoi(split_strings[SERIAL_IDX]);
            momsn_         = std::stoi(split_strings[MOMSN_IDX]);
            transmit_time_ = split_strings[TIME_IDX];
            lat_           = std::stof(split_strings[LAT_IDX]);
            lon_           = std::stof(split_strings[LON_IDX]);
            cep_           = std::stoi(split_strings[CEP_IDX]);
            data_          = split_strings[DATA_IDX];
        }
    };

    beast::flat_buffer                 buf_{static_cast<std::size_t>(MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES * 2)};
    tcp::socket                        socket_;
    http::request<http::dynamic_body>  req_;
    http::response<http::dynamic_body> res_;

    void readReq()
    {
        http::async_read(socket_, buf_, req_, [this](beast::error_code ec, std::size_t /*bytesTransferred*/) {
            if (!ec) {
                processReq();
            }
        });
    }

    void processReq()
    {
        res_.version(req_.version());
        res_.keep_alive(false);  // check this

        switch (req_.method()) {
            case http::verb::post:
                doPost();
                break;
            case http::verb::get:
            default:;
        }
    }

    // https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
    // IMPORTANT: Have 3 seconds to send HTTP status 200
    void doPost()
    {
        beast::string_view content_type = req_["content-type"];
        if (content_type == "application/x-www-form-urlencoded") {
            res_.result(http::status::ok);
            writeRes();  // non-blocking
            std::string query_string = beast::buffers_to_string(req_.body().data());
            MOMsgParams params       = MOMsgParams(query_string);
        }
    }

    void writeRes()
    {
        http::async_write(socket_, res_, [this](beast::error_code ec, std::size_t /*bytesWritten*/) {
            socket_.shutdown(tcp::socket::shutdown_send, ec);
        });
    }
};

void listen(tcp::acceptor & acceptor, tcp::socket & socket)
{
    acceptor.async_accept(socket, [&](beast::error_code ec) {
        if (!ec) {
            std::make_shared<HTTPServer>(std::move(socket))->run();
        }
        listen(acceptor, socket);
    });
}

int main()
{
    // TODO(): Run parameters not clearly defined yet.
    // will need to switch "admin" with the relevant db and
    // make the connection string an input parameter to differentiate
    // between local testing and deployment at runtime
    SailbotDB sailbot_db("admin", "mongodb://localhost:27017");
    if (!sailbot_db.testConnection()) {
        std::cout << "Failed to connect to database, exiting" << std::endl;
        return -1;
    }

    bio::ip::address address = bio::ip::make_address("127.0.0.1");
    const uint32_t   port    = 8081;
    bio::io_context  io;
    tcp::acceptor    acceptor{io, {address, port}};
    tcp::socket      socket{io};

    listen(acceptor, socket);

    io.run();

    return 0;
}
