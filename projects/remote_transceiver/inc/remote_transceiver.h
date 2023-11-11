#pragma once

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <memory>

#include "cmn_hdrs/shared_constants.h"
#include "sailbot_db.h"

static const std::string TESTING_HOST = "127.0.0.1";
constexpr uint32_t       TESTING_PORT = 8081;
constexpr int            HTTP_VERSION = 11;  // HTTP v1.1

namespace beast = boost::beast;
namespace http  = beast::http;
namespace bio   = boost::asio;
using tcp       = boost::asio::ip::tcp;

namespace remote_transceiver
{
struct MOMsgParams
{
    using Params = struct
    {
        uint64_t    imei_;
        uint32_t    serial_;  // Don't know the max size
        uint16_t    momsn_;
        std::string transmit_time_;  // UTC date and time. Ex: "21-10-31 10:41:50"
        float       lat_;
        float       lon_;
        uint32_t    cep_;   // estimate of the accuracy (in km) of the reported lat_ lon_ fields
        std::string data_;  // hex-encoded
    };
    Params params_;

    /**
         * @brief Construct a new MOMsg object
         *
         * @param query_string
         */
    // Example: ?imei=1234&serial=5678&momsn=9123&transmit_time=21-10-31 10:41:50&iridium_latitude=12.34&iridium_longitude=56.78&iridium_cep=2&data=A1B2C3
    explicit MOMsgParams(const std::string & query_string);

    explicit MOMsgParams(Params params) : params_(params) {}
};

class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    explicit HTTPServer(tcp::socket socket, SailbotDB db);
    void        run();
    static void runServer(tcp::acceptor & acceptor, tcp::socket & socket, SailbotDB & db);

private:
    beast::flat_buffer                 buf_{static_cast<std::size_t>(MAX_LOCAL_TO_REMOTE_PAYLOAD_SIZE_BYTES * 2)};
    tcp::socket                        socket_;
    http::request<http::dynamic_body>  req_;
    http::response<http::dynamic_body> res_;
    SailbotDB                          db_;

    void readReq();
    void processReq();
    void doBadReq();
    void doPost();
    void doGet();
    void writeRes();
};

namespace http_client
{
struct ConnectionInfo
{
    std::string host;
    std::string port;
    std::string target;

    std::tuple<std::string &, std::string &, std::string &> get() { return {host, port, target}; }
};
std::pair<http::status, std::string> get(ConnectionInfo info);
http::status                         post(ConnectionInfo info, std::string content_type, const std::string & body);
}  // namespace http_client

}  // namespace remote_transceiver
