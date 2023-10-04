#include <cstdint>
#include <random>

#include "bsoncxx/builder/basic/document.hpp"
#include "gtest/gtest.h"
#include "mongocxx/collection.hpp"
#include "mongocxx/cursor.hpp"
#include "sailbot_db.h"
#include "sensors.pb.h"
#include "shared_constants.h"

static constexpr int NUM_AIS_SHIPS = 15;

using Placeholder::Sensors;

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
        mongocxx::collection ais_coll = db_[COLLECTION_AIS_SHIPS];
        gps_coll.delete_many(bsoncxx::builder::basic::make_document());
        ais_coll.delete_many(bsoncxx::builder::basic::make_document());
    }

    /**
     * @brief Read all sensors from the TestDB
     *
     * @return Sensors object
     */
    Sensors dumpSensors()
    {
        Sensors sensors;

        // Protobuf handles freeing of dynamically allocated objects automatically
        // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks)

        mongocxx::collection gps_coll = db_[COLLECTION_GPS];
        mongocxx::cursor     gps_docs = gps_coll.find({});
        EXPECT_EQ(gps_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        Sensors::Gps * gps = new Sensors::Gps();
        sensors.set_allocated_gps(gps);
        bsoncxx::document::view gps_doc = *(gps_docs.begin());
        gps->set_latitude(static_cast<float>(gps_doc["latitude"].get_double().value));
        gps->set_longitude(static_cast<float>(gps_doc["longitude"].get_double().value));
        gps->set_speed(static_cast<float>(gps_doc["speed"].get_double().value));
        gps->set_heading(static_cast<float>(gps_doc["heading"].get_double().value));

        mongocxx::collection ais_coll = db_[COLLECTION_AIS_SHIPS];
        mongocxx::cursor     ais_docs = ais_coll.find({});
        EXPECT_EQ(ais_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view ais_ships_doc = *(ais_docs.begin());
        for (bsoncxx::array::element ais_ships_doc : ais_ships_doc["ais_ships"].get_array().value) {
            Sensors::Ais * ais_ship = sensors.add_ais_ships();
            ais_ship->set_id(static_cast<uint32_t>(ais_ships_doc["id"].get_int64().value));
            ais_ship->set_latitude(static_cast<float>(ais_ships_doc["latitude"].get_double().value));
            ais_ship->set_longitude(static_cast<float>(ais_ships_doc["longitude"].get_double().value));
            ais_ship->set_speed(static_cast<float>(ais_ships_doc["speed"].get_double().value));
            ais_ship->set_heading(static_cast<float>(ais_ships_doc["heading"].get_double().value));
        }
        EXPECT_EQ(sensors.ais_ships().size(), NUM_AIS_SHIPS) << "Size mismatch when reading AIS ships from DB";

        return sensors;

        // NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)
    }
};

static TestDB             g_test_db   = TestDB();              // initialize the TestDB instance
static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                   // initialize random number generator with seed

class TestRemoteTransceiver : public ::testing::Test
{
protected:
    TestRemoteTransceiver() { g_test_db.cleanDB(); }
    ~TestRemoteTransceiver() override {}
};

/**
 * @brief Generate random GPS data
 *
 * @return pointer to generated GPS data
 */
Sensors::Gps * genRandGpsData()
{
    Sensors::Gps * gps_data = new Sensors::Gps;

    std::uniform_real_distribution<float> lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float> speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float> heading_dist(HEADING_LBND, HEADING_UBND);
    gps_data->set_latitude(lat_dist(g_mt));
    gps_data->set_longitude(lon_dist(g_mt));
    gps_data->set_speed(speed_dist(g_mt));
    gps_data->set_heading(heading_dist(g_mt));

    return gps_data;
}

void genRandAisData(Sensors::Ais * ais_ship)
{
    std::uniform_int_distribution<uint32_t> id_dist(0, UINT32_MAX);
    std::uniform_real_distribution<float>   lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float>   lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float>   speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);

    ais_ship->set_id(id_dist(g_mt));
    ais_ship->set_latitude(lat_dist(g_mt));
    ais_ship->set_longitude(lon_dist(g_mt));
    ais_ship->set_speed(speed_dist(g_mt));
    ais_ship->set_heading(heading_dist(g_mt));
}

/**
 * @brief Generate random data for all sensors
 *
 * @return Sensors object
 */
Sensors genRandSensors()
{
    Sensors sensors;
    // Protobuf handles freeing of dynamically allocated objects automatically
    // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks)
    sensors.set_allocated_gps(genRandGpsData());
    // TODO(): Polaris should be included as one of the AIS ships
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        genRandAisData(sensors.add_ais_ships());
    }
    return sensors;
    // NOLINTEND(clang-analyzer-cplusplus.NewDeleteLeaks)
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
    Sensors rand_sensors = genRandSensors();
    ASSERT_TRUE(g_test_db.storeSensors(rand_sensors));
    Sensors dumped_sensors = g_test_db.dumpSensors();

    EXPECT_FLOAT_EQ(dumped_sensors.gps().latitude(), rand_sensors.gps().latitude());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().longitude(), rand_sensors.gps().longitude());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().speed(), rand_sensors.gps().speed());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().heading(), rand_sensors.gps().heading());

    // Array size checking done in dumpSensors
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        const Sensors::Ais & dumped_ais_ships = dumped_sensors.ais_ships(i);
        const Sensors::Ais & rand_ais_ships   = rand_sensors.ais_ships(i);
        EXPECT_EQ(dumped_ais_ships.id(), rand_ais_ships.id());
        EXPECT_FLOAT_EQ(dumped_ais_ships.latitude(), rand_ais_ships.latitude());
        EXPECT_FLOAT_EQ(dumped_ais_ships.longitude(), rand_ais_ships.longitude());
        EXPECT_FLOAT_EQ(dumped_ais_ships.speed(), rand_ais_ships.speed());
        EXPECT_FLOAT_EQ(dumped_ais_ships.heading(), rand_ais_ships.heading());
    }
}
