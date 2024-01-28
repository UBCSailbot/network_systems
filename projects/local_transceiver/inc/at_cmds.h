#pragma once

// Full command set: https://cdn-shop.adafruit.com/product-files/4521/4521-AT%20command.pdf
// Section numbers in this header file refer to this document

#include <cstdint>
#include <string>

namespace AT
{

const std::string DELIMITER = "\r\n";
const std::string STATUS_OK = "OK";
const std::string RSP_READY = "READY";

const std::string CHECK_CONN  = "AT";
const std::string SBD_SESSION = "AT+SBDIX";  // 5.144

namespace write_bin  // 5.154
{
const std::string CMD = "AT+SBDWB=";

namespace rsp
{
const std::string SUCCESS      = "0";
const std::string TIMEOUT      = "1";
const std::string BAD_CHECKSUM = "2";
const std::string WRONG_SIZE   = "3";
}  // namespace rsp
}  // namespace write_bin

/**
 * @brief Simple Line struct to help enforce DRY when dealing with strings while performing reads and writes
 *
 */
struct Line
{
    /**
     * @param str valid AT command or response string
     */
    inline explicit Line(const std::string & str) : str_(str == AT::DELIMITER || str == "\n" ? str : str + "\r") {}
    // In most cases, str_ will just be the input str to the constructor
    // AT::DELIMITER and \n are exceptions, and remain the same
    const std::string str_;
};

/**
 * Struct representing the response to the CHECK_STATUS command
 * 5.144
 */
struct SBDStatusRsp  // TODO(Jng468): Implement this class
{
    static constexpr uint8_t MO_SUCCESS_START = 0;
    static constexpr uint8_t MO_SUCCESS_END   = 4;

    uint8_t  MO_status_;
    uint16_t MOMSN_;
    uint8_t  MT_status_;
    uint16_t MTMSN_;
    uint8_t  MT_len_;
    uint8_t  MT_queued_;

    /**
     * @brief Construct a new Status Response object
     *
     * @param rsp_string string of format "+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>""
     */
    explicit SBDStatusRsp(const std::string & rsp_string)
    {
        (void)rsp_string;
        MO_status_ = 0;
        MOMSN_     = 0;
        MT_status_ = 0;
        MTMSN_     = 0;
        MT_len_    = 0;
        MT_queued_ = 0;
    };

    /**
     * @brief Check if last Mobile Originated (i.e. transmitted sensors) transaction was successful
     *
     * @return true  on success
     * @return false on failure
     */
    bool MOSuccess() const { return MO_status_ <= MO_SUCCESS_END; }
};

}  // namespace AT
