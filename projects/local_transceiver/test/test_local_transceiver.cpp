#include <boost/process.hpp>
#include <boost/process/system.hpp>

#include "gtest/gtest.h"
#include "local_transceiver.h"

namespace bp = boost::process;

class VirtualIridium
{
public:
    VirtualIridium() { server_proc_ = bp::child(RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH); }
    ~VirtualIridium() { shutdown(); }

    void shutdown() { server_proc_.terminate(); }

private:
    bp::child server_proc_;
};

static VirtualIridium g_server_ = VirtualIridium();

class TestLocalTransceiver : public ::testing::Test
{
protected:
    static void TearDownTestSuite() { g_server_.shutdown(); }

    // TestLocalTransceiver() { lcl_trns_ = LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT); }
    TestLocalTransceiver() : lcl_trns_(LocalTransceiver(LOCAL_TRANSCEIVER_TEST_PORT)) {}
    ~TestLocalTransceiver() override {}

    LocalTransceiver lcl_trns_;
};

TEST_F(TestLocalTransceiver, Test) {}
