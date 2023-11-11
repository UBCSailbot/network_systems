#include "remote_transceiver.h"

#include <boost/algorithm/string.hpp>
#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <boost/beast/core/buffers_to_string.hpp>
#include <boost/beast/core/error.hpp>
#include <boost/beast/core/flat_buffer.hpp>
#include <boost/beast/core/string_type.hpp>
#include <boost/beast/http.hpp>
#include <boost/core/ignore_unused.hpp>
#include <iostream>
#include <memory>
#include <string>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db.h"
#include "sensors.pb.h"

namespace beast = boost::beast;
namespace http  = beast::http;
namespace bio   = boost::asio;
using tcp       = boost::asio::ip::tcp;

class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    explicit HTTPServer(tcp::socket socket, SailbotDB db) : socket_(std::move(socket)), db_(std::move(db)) {}
    void run() { readReq(); }

private:
    struct MOMsgParams
    {
        uint64_t    imei_;
        uint32_t    serial_;  // Don't know the max size
        uint16_t    momsn_;
        std::string transmit_time_;  // UTC date and time. Ex: "21-10-31 10:41:50"
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
    SailbotDB                          db_;

    void readReq()
    {
        std::shared_ptr<HTTPServer> self = shared_from_this();
        http::async_read(socket_, buf_, req_, [self](beast::error_code e, std::size_t /*bytesTransferred*/) {
            if (!e) {
                self->processReq();
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
                doGet();
                break;
            default:
                doBadReq();
        }
        writeRes();
    }

    void doBadReq()
    {
        res_.result(http::status::bad_request);
        res_.set(http::field::content_type, "text/plain");
        beast::ostream(res_.body()) << "Invalid request method: " << req_.method_string();
    }

    // https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
    // IMPORTANT: Have 3 seconds to send HTTP status 200, so do not process data on same thread before responding
    void doPost()
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

    void doGet()
    {
        res_.result(http::status::ok);
        res_.set(http::field::server, "Sailbot Remote Transceiver");
        res_.set(http::field::content_type, "text/plain");
        beast::ostream(res_.body()) << "PLACEHOLDER\r\n";
    }

    void writeRes()
    {
        res_.set(http::field::content_length, std::to_string(res_.body().size()));

        std::shared_ptr<HTTPServer> self = shared_from_this();
        http::async_write(socket_, res_, [self](beast::error_code e, std::size_t /*bytesWritten*/) {
            self->socket_.shutdown(tcp::socket::shutdown_send, e);
        });
    }
};

void listen(tcp::acceptor & acceptor, tcp::socket & socket, SailbotDB & db)
{
    acceptor.async_accept(socket, [&](beast::error_code e) {
        if (!e) {
            std::make_shared<HTTPServer>(std::move(socket), std::move(db))->run();
        }
        listen(acceptor, socket, db);
    });
}

int main()
{
    // TODO(): Run parameters not clearly defined yet.
    // will need to switch "admin" with the relevant db
    SailbotDB sailbot_db("admin", MONGODB_CONN_STR);
    if (!sailbot_db.testConnection()) {
        std::cout << "Failed to connect to database, exiting" << std::endl;
        return -1;
    }

    static const std::string TESTING_ADDR = "127.0.0.1";
    constexpr uint32_t       TESTING_PORT = 8081;

    // TODO(): Need to distinguish between test and deployment address + port
    bio::ip::address addr = bio::ip::make_address(TESTING_ADDR);
    const uint32_t   port = TESTING_PORT;

    bio::io_context io;
    tcp::acceptor   acceptor{io, {addr, port}};
    tcp::socket     socket{io};

    listen(acceptor, socket, sailbot_db);

    io.run();

    return 0;
}
