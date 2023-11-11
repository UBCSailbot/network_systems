#pragma once

#include <boost/algorithm/string.hpp>
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
class HTTPServer : public std::enable_shared_from_this<HTTPServer>
{
public:
    explicit HTTPServer(tcp::socket socket, SailbotDB db);
    void        run();
    static void runServer(tcp::acceptor & acceptor, tcp::socket & socket, SailbotDB & db);

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

    void readReq();
    void processReq();
    void doBadReq();
    void doPost();
    void doGet();
    void writeRes();
};

namespace http_client
{
std::pair<http::status, std::string> get(
  const std::string & host, const std::string & port, const std::string & target);
http::status post(std::string content_type, const std::string & data);
}  // namespace http_client

}  // namespace remote_transceiver
