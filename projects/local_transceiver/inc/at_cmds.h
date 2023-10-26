#pragma once

// Full command set: https://cdn-shop.adafruit.com/product-files/4521/4521-AT%20command.pdf
// Section numbers in this header file refer to this document

#include <cstdint>
#include <string>

namespace AT
{
const std::string DELIMITER = "\r";
const std::string STATUS_OK = "OK";

const std::string CHECK_CONN  = "AT" + DELIMITER;
const std::string SBD_SESSION = "AT+SBDIX" + DELIMITER;  // 5.144

/**
 * Class representing the response to the CHECK_STATUS command
 * 5.144
 */
class SBDStatusResponse  // TODO(Jng468): Implement this class
{
public:
    /**
     * @brief Construct a new Status Response object
     *
     * @param rsp_string string of format "+SBDIX:<MO status>,<MOMSN>,<MT status>,<MTMSN>,<MT length>,<MTqueued>""
     */
    explicit SBDStatusResponse(const std::string & rsp_string);

    /**
     * @brief Check if last Mobile Originated (i.e. transmitted sensors) transaction was successful
     *
     * @return true  on success
     * @return false on failure
     */
    inline bool MOSuccess() const { return MO_status_ < MO_SUCCESS_END; }

private:
    static constexpr uint8_t MO_SUCCESS_START = 0;
    static constexpr uint8_t MO_SUCCESS_END   = 5;
    uint8_t                  MO_status_;
    uint16_t                 MOMSN_;
    uint8_t                  MT_status_;
    uint16_t                 MTMSN_;
    uint8_t                  MT_len_;
    uint8_t                  MT_queued_;
};

}  // namespace AT
