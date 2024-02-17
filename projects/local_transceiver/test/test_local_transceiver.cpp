/* IMPORTANT: Make sure only one instance of network_systems/scripts/run_virtual_iridium.sh is running */

#include <gtest/gtest.h>

#include <boost/process.hpp>
#include <boost/system/system_error.hpp>
#include <fstream>

#include "at_cmds.h"
#include "cmn_hdrs/shared_constants.h"
#include "local_transceiver.h"
#include "sensors.pb.h"

namespace bp = boost::process;

/* >>>>>README<<<<<<
Local Transceiver unit tests rely on two other programs: Virtual Iridium and HTTP Echo Server
1. Spawning a separate process for RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH doesn't work very well because the script
    spawns it's own subprocess for mock serial ports. You can spawn it ezpz, but cleaning up mock serial port
    subprocesses is a lot harder than you'd think. Hence, it's not done in the test code.
2. Virtual Iridium needs a valid HTTP POST endpoint for certain commands. RUN_HTTP_ECHO_SERVER_CMD runs a simple
   server that just echos whatever it receives.
   ***IMPORTANT***: Make sure the echo server is running on the host and port specified in the virtual iridium
                    --webhook_server_endpoint argument (default: 127.0.0.1:8081)
*/
class TestLocalTransceiver : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { http_echo_server_proc_ = bp::child(RUN_HTTP_ECHO_SERVER_CMD); }

    static void TearDownTestSuite() { http_echo_server_proc_.terminate(); }

    TestLocalTransceiver()
    {
        try {
            lcl_trns_ = new LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT, SATELLITE_BAUD_RATE);
        } catch (boost::system::system_error & /**/) {
            std::cerr << "Failed to create Local Transceiver for tests, is only one instance of: \""
                      << RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH << "\" running?" << std::endl;
        }
    }
    ~TestLocalTransceiver() override
    {
        lcl_trns_->stop();
        delete lcl_trns_;
    }

    LocalTransceiver * lcl_trns_;

private:
    static bp::child http_echo_server_proc_;
};
bp::child TestLocalTransceiver::http_echo_server_proc_ = {};

/**
 * @brief Verify debugSend
 */
TEST_F(TestLocalTransceiver, debugSendTest)
{
    EXPECT_EQ(lcl_trns_->debugSend(AT::CHECK_CONN), AT::Line(AT::STATUS_OK).str_);
}

/**
 * @brief Send a binary string to virtual_iridium and verify it is received
 *        Uses gps custom interface
 */
TEST_F(TestLocalTransceiver, sendGpsTest)
{
    constexpr float holder = 14.3;  // arbitrary number for testing

    custom_interfaces::msg::GPS gps;
    gps.heading.set__heading(holder);
    gps.lat_lon.set__latitude(holder);
    gps.lat_lon.set__longitude(holder);
    gps.speed.set__speed(holder);
    lcl_trns_->updateSensor(gps);
    lcl_trns_->send();
}

/**
 * @brief Open LOCAL_TRANSCEIVER_TEST_PORT as a file and visually confirm response
 *
 */
TEST_F(TestLocalTransceiver, visualVerification)
{
    constexpr float holder = 14.3;

    custom_interfaces::msg::GPS gps;
    gps.heading.set__heading(holder);
    gps.lat_lon.set__latitude(holder);
    gps.lat_lon.set__longitude(holder);
    gps.speed.set__speed(holder);
    lcl_trns_->updateSensor(gps);
    lcl_trns_->send();

    //std::ifstream port(LOCAL_TRANSCEIVER_TEST_PORT);
    std::string portline;

    //while (std::getline(port, portline)) {
    // Output the content of portline
    // std::cout << "Read from file: " << portline << std::endl;
    //}

    // port.close();

    /*std::ofstream mockSerialFile(LOCAL_TRANSCEIVER_TEST_PORT);

    // Redirect cout to the file
    std::streambuf *originalCoutBuffer = std::cout.rdbuf();
    std::cout.rdbuf(mockSerialFile.rdbuf());

    // Execute the function that generates output
    lcl_trns_->receive();

    // Restore the original cout buffer
    std::cout.rdbuf(originalCoutBuffer);
    */
}
