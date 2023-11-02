/* IMPORTANT: Make sure only one instance of network_systems/scripts/run_virtual_iridium.sh is running */

#include <boost/system/system_error.hpp>
#include "cmn_hdrs/shared_constants.h"
#include "gtest/gtest.h"
#include "local_transceiver.h"

class TestLocalTransceiver : public ::testing::Test
{
protected:

    // TestLocalTransceiver() : lcl_trns_(LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT, SATELLITE_BAUD_RATE)) {}
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
    ~TestLocalTransceiver() override { delete lcl_trns_; }

    LocalTransceiver * lcl_trns_;
};

TEST_F(TestLocalTransceiver, PlaceholderTest) { std::cout << "PLACEHOLDER" << std::endl; }

TEST_F(TestLocalTransceiver, PlaceholderTest2) { std::cout << "PLACEHOLDER" << std::endl; }
