#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <thread>

#include "cmn_hdrs/shared_constants.h"
#include "gtest/gtest.h"
#include "local_transceiver.h"
#include "virtual_iridium_server/virtual_iridium_server.h"

// Start a common virtual iridium server for all tests
static VirtualIridiumServer g_server_ = VirtualIridiumServer();

class TestLocalTransceiver : public ::testing::Test
{
protected:
    static void SetUpTestSuit()
    {
        // Register TearDownTestSuite() as the exit handler to kill spawned virtual iridium server on any exit
        // int result = std::atexit(TearDownTestSuite);
        // if (result != 0) {
        //     throw std::runtime_error("Failed to register exit handler");
        // }
    }

    // Kill virtual iridium server when tests are done
    static void TearDownTestSuite()
    {
        g_server_.shutdown();
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    TestLocalTransceiver() : lcl_trns_(LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT, SATELLITE_BAUD_RATE)) {}
    ~TestLocalTransceiver() override {}

    LocalTransceiver lcl_trns_;
};

TEST_F(TestLocalTransceiver, PlaceholderTest) { std::cout << "PLACEHOLDER" << std::endl; }

TEST_F(TestLocalTransceiver, PlaceholderTest2) { std::cout << "PLACEHOLDER" << std::endl; }
