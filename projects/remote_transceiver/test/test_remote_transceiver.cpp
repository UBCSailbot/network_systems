#include <random>

#include "bsoncxx/builder/basic/document.hpp"
#include "gtest/gtest.h"
#include "mongocxx/collection.hpp"
#include "mongocxx/cursor.hpp"
#include "remote_transceiver.h"
#include "sailbot_db.h"
#include "sensors.pb.h"
#include "shared_constants.h"

using bsoncxx::builder::basic::make_document;

/**
 * Child class of SailbotDB that includes additional database utility functions to help testing
 */
class TestDB : public SailbotDB
{
public:
    static constexpr const char * TEST_DB = "test";

    TestDB() : SailbotDB(TEST_DB) {}

    /**
     * @brief Delete all documents in all collections
     */
    void cleanDB()
    {
        mongocxx::collection gps_coll = db_[COLLECTION_GPS];
        gps_coll.delete_many(make_document());
    }

    /**
     * @brief Read all sensors from the TestDB
     *
     * @return Sensors object
     */
    Placeholder::Sensors dumpSensors()
    {
        Placeholder::Sensors sensors;
        mongocxx::collection gps_coll = db_[COLLECTION_GPS];
        mongocxx::cursor     gps_docs = gps_coll.find({});
        EXPECT_EQ(gps_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        Placeholder::Sensors::Gps * gps = new Placeholder::Sensors::Gps();
        sensors.set_allocated_gps(gps);
        bsoncxx::document::view gps_doc = *(gps_docs.begin());
        gps->set_lat(static_cast<float>(gps_doc["lat"].get_double().value));
        gps->set_lon(static_cast<float>(gps_doc["lon"].get_double().value));
        gps->set_speed(static_cast<float>(gps_doc["speed"].get_double().value));
        gps->set_heading(static_cast<float>(gps_doc["heading"].get_double().value));
        // Protobuf handles freeing of dynamically allocated objects (gps) automatically
        return sensors;  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
    }
};

static TestDB             g_test_db   = TestDB();              // initialize the TestDB instance
static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                   // initialize random number generator with seed

class TestRemoteTransceiver : public ::testing::Test
{
protected:
    TestRemoteTransceiver() {}
    ~TestRemoteTransceiver() override { g_test_db.cleanDB(); }
};

/**
 * @brief Generate random GPS data
 *
 * @return pointer to generated GPS data
 */
Placeholder::Sensors::Gps * genRandGpsData()
{
    Placeholder::Sensors::Gps * gps_data = new Placeholder::Sensors::Gps;

    std::uniform_real_distribution<float> lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float> speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float> heading_dist(HEADING_LBND, HEADING_UBND);
    gps_data->set_lat(lat_dist(g_mt));
    gps_data->set_lon(lon_dist(g_mt));
    gps_data->set_speed(speed_dist(g_mt));
    gps_data->set_heading(heading_dist(g_mt));

    return gps_data;
}

/**
 * @brief Generate random data for all sensors
 *
 * @return Sensors object
 */
Placeholder::Sensors genRandSensors()
{
    Placeholder::Sensors sensors;
    sensors.set_allocated_gps(genRandGpsData());
    return sensors;  // NOLINT(clang-analyzer-cplusplus.NewDeleteLeaks)
}

/**
 * @brief Check that MongoDB is running
 */
TEST_F(TestRemoteTransceiver, TestConnection)
{
    ASSERT_TRUE(g_test_db.testConnection()) << "MongoDB not running - remember to connect!";
}

/**
 * @brief Write random sensor data to the TestDB - read and verify said data
 */
TEST_F(TestRemoteTransceiver, TestStoreSensors)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    Placeholder::Sensors rand_sensors = genRandSensors();
    ASSERT_TRUE(g_test_db.storeSensors(rand_sensors));
    Placeholder::Sensors dumped_sensors = g_test_db.dumpSensors();
    EXPECT_FLOAT_EQ(dumped_sensors.gps().lat(), rand_sensors.gps().lat());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().lon(), rand_sensors.gps().lon());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().speed(), rand_sensors.gps().speed());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().heading(), rand_sensors.gps().heading());
}
