#include <cstdint>
#include <random>

#include "bsoncxx/builder/basic/document.hpp"
#include "gtest/gtest.h"
#include "mongocxx/collection.hpp"
#include "mongocxx/cursor.hpp"
#include "sailbot_db.h"
#include "sensors.pb.h"
#include "shared_constants.h"

static constexpr int  NUM_AIS_SHIPS       = 15; // arbitrary number
static constexpr int  NUM_GENERIC_SENSORS = 5;  //arbitrary number
static constexpr int  NUM_BATTERIES       = 2;
static constexpr int  NUM_WIND_SENSORS    = 2;
static constexpr int  NUM_PATH_WAYPOINTS  = 5;  //arbitrary number
static constexpr auto MONGODB_CONN_STR    = "mongodb://localhost:27017";

using Polaris::Sensors;

//Child class of SailbotDB that includes additional database utility functions to help testing
class TestDB : public SailbotDB
{
public:
    static constexpr const char * TEST_DB = "test";
    TestDB() : SailbotDB(TEST_DB, MONGODB_CONN_STR) {}

    /**
     * @brief Delete all documents in all collections
     */
    void cleanDB()
    {
        mongocxx::collection gps_coll       = db_[COLLECTION_GPS];
        mongocxx::collection ais_coll       = db_[COLLECTION_AIS_SHIPS];
        mongocxx::collection generic_coll   = db_[COLLECTION_DATA_SENSORS];
        mongocxx::collection batteries_coll = db_[COLLECTION_BATTERIES];
        mongocxx::collection wind_coll      = db_[COLLECTION_WIND_SENSORS];
        mongocxx::collection path_coll      = db_[COLLECTION_PATH];

        gps_coll.delete_many(bsoncxx::builder::basic::make_document());
        ais_coll.delete_many(bsoncxx::builder::basic::make_document());
        generic_coll.delete_many(bsoncxx::builder::basic::make_document());
        batteries_coll.delete_many(bsoncxx::builder::basic::make_document());
        wind_coll.delete_many(bsoncxx::builder::basic::make_document());
        path_coll.delete_many(bsoncxx::builder::basic::make_document());
    }

    /**
     * @return Sensors objects: gps, ais, generic, batteries, wind, path
     */
    Sensors dumpSensors()
    {
        Sensors sensors;
        // Protobuf handles freeing of dynamically allocated objects automatically
        // NOLINTBEGIN(clang-analyzer-cplusplus.NewDeleteLeaks)

        // gps
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

        // ais ships
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
            ais_ship->set_rot(static_cast<float>(ais_ships_doc["rot"].get_double().value));
            ais_ship->set_width(static_cast<float>(ais_ships_doc["width"].get_double().value));
            ais_ship->set_length(static_cast<float>(ais_ships_doc["length"].get_double().value));
        }
        EXPECT_EQ(sensors.ais_ships().size(), NUM_AIS_SHIPS) << "Size mismatch when reading AIS ships from DB";

        // generic sensor
        mongocxx::collection generic_coll       = db_[COLLECTION_DATA_SENSORS];
        mongocxx::cursor     generic_sensor_doc = generic_coll.find({});
        EXPECT_EQ(generic_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view generic_doc = *(generic_sensor_doc.begin());
        for (bsoncxx::array::element generic_doc : generic_doc["data_sensors"].get_array().value) {
            Sensors::Generic * generic = sensors.add_data_sensors();
            generic->set_id(static_cast<uint8_t>(generic_doc["id"].get_int32().value));
            generic->set_data(static_cast<uint64_t>(generic_doc["data"].get_int64().value));
        }

        // battery
        mongocxx::collection batteries_coll     = db_[COLLECTION_BATTERIES];
        mongocxx::cursor     batteries_data_doc = batteries_coll.find({});
        EXPECT_EQ(batteries_coll.count_documents({}), 1)
          << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view batteries_doc = *(batteries_data_doc.begin());
        for (bsoncxx::array::element batteries_doc : batteries_doc["batteries"].get_array().value) {
            Sensors::Battery * battery = sensors.add_batteries();
            battery->set_voltage(static_cast<float>(batteries_doc["voltage"].get_double().value));
            battery->set_current(static_cast<float>(batteries_doc["current"].get_double().value));
        }
        EXPECT_EQ(sensors.batteries().size(), NUM_BATTERIES) << "Size mismatch when reading batteries from DB";

        // wind sensor
        mongocxx::collection wind_coll        = db_[COLLECTION_WIND_SENSORS];
        mongocxx::cursor     wind_sensors_doc = wind_coll.find({});
        EXPECT_EQ(wind_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view wind_doc = *(wind_sensors_doc.begin());
        for (bsoncxx::array::element wind_doc : wind_doc["wind_sensors"].get_array().value) {
            Sensors::Wind * wind = sensors.add_wind_sensors();
            wind->set_speed(static_cast<float>(wind_doc["speed"].get_double().value));
            wind->set_direction(static_cast<int16_t>(wind_doc["direction"].get_int32().value));
        }
        EXPECT_EQ(sensors.wind_sensors().size(), NUM_WIND_SENSORS) << "Size mismatch when reading batteries from DB";

        // local path
        mongocxx::collection path_coll      = db_[COLLECTION_PATH];
        mongocxx::cursor     local_path_doc = path_coll.find({});
        EXPECT_EQ(path_coll.count_documents({}), 1) << "Error: Test DB should only have one document per collection";
        bsoncxx::document::view path_doc = *(local_path_doc.begin());
        for (bsoncxx::array::element path_doc : path_doc["local_path_data"].get_array().value) {
            Sensors::Path * path = sensors.add_local_path_data();
            path->set_latitude(static_cast<float>(path_doc["latitude"].get_double().value));
            path->set_longitude(static_cast<float>(path_doc["longitude"].get_double().value));
        }
        EXPECT_EQ(sensors.local_path_data().size(), NUM_PATH_WAYPOINTS)
          << "Size mismatch when reading path waypoints from DB";

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
 * @brief generate random GPS data
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

/**
 * @brief generate random ais ships data
 *
 * @param ais_ship pointer to generated ais data
 */
void genRandAisData(Sensors::Ais * ais_ship)
{
    std::uniform_int_distribution<uint32_t> id_dist(0, UINT32_MAX);
    std::uniform_real_distribution<float>   lat_dist(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float>   lon_dist(LON_LBND, LON_UBND);
    std::uniform_real_distribution<float>   speed_dist(SPEED_LBND, SPEED_UBND);
    std::uniform_real_distribution<float>   heading_dist(HEADING_LBND, HEADING_UBND);
    std::uniform_real_distribution<float>   rot_dist(ROT_LBND, ROT_UBND);
    std::uniform_real_distribution<float>   width_dist(DIMENSION_LBND, DIMENSION_UBND);
    std::uniform_real_distribution<float>   length_dist(DIMENSION_LBND, DIMENSION_UBND);

    ais_ship->set_id(id_dist(g_mt));
    ais_ship->set_latitude(lat_dist(g_mt));
    ais_ship->set_longitude(lon_dist(g_mt));
    ais_ship->set_speed(speed_dist(g_mt));
    ais_ship->set_heading(heading_dist(g_mt));
    ais_ship->set_rot(rot_dist(g_mt));
    ais_ship->set_width(width_dist(g_mt));
    ais_ship->set_length(length_dist(g_mt));
}

/**
 * @brief generate random generic sensor data
 *
 * @return pointer to generated generic sensor data
 */
void genRandGenericSensorData(Sensors::Generic * generic_sensor)
{
    std::uniform_int_distribution<uint8_t>  id_generic(0, UINT8_MAX);
    std::uniform_int_distribution<uint64_t> data_generic(0, UINT64_MAX);

    generic_sensor->set_id(id_generic(g_mt));
    generic_sensor->set_data(data_generic(g_mt));
}

/**
 * @brief generate random battery data
 *
 * @return pointer to generated battery data
 */
void genRandBatteriesData(Sensors::Battery * battery)
{
    std::uniform_real_distribution<float> voltage_battery(VOLT_LBND, VOLT_UBND);
    std::uniform_real_distribution<float> current_battery(CURRENT_LBND, CURRENT_UBND);

    battery->set_voltage(voltage_battery(g_mt));
    battery->set_current(current_battery(g_mt));
}

/**
 * @brief generate random wind sensors data
 *
 * @return pointer to generated wind sensors data
 */
void genRandWindData(Sensors::Wind * wind_data)
{
    std::uniform_real_distribution<float> speed_wind(SPEED_LBND, SPEED_UBND);
    std::uniform_int_distribution<int>    direction_wind(DIRECTION_LBND, DIRECTION_UBND);

    wind_data->set_speed(speed_wind(g_mt));
    wind_data->set_direction(direction_wind(g_mt));
}

/**
 * @brief generate random path data
 *
 * @return pointer to generated path data
 */
void genRandPathData(Sensors::Path * path_data)
{
    std::uniform_real_distribution<float> latitude_path(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> longitude_path(LON_LBND, LON_UBND);

    path_data->set_latitude(latitude_path(g_mt));
    path_data->set_longitude(longitude_path(g_mt));
}

/**
 * @brief generate random generic sensor data
 *
 * @param generic_sensor pointer to generated generic sensor data
 */
void genRandGenericSensorData(Sensors::Generic * generic_sensor)
{
    std::uniform_int_distribution<uint8_t>  id_generic(0, UINT8_MAX);
    std::uniform_int_distribution<uint64_t> data_generic(0, UINT64_MAX);

    generic_sensor->set_id(id_generic(g_mt));
    generic_sensor->set_data(data_generic(g_mt));
}

/**
 * @brief generate random battery data
 *
 * @param battery pointer to generated battery data
 */
void genRandBatteriesData(Sensors::Battery * battery)
{
    std::uniform_real_distribution<float> voltage_battery(VOLT_LBND, VOLT_UBND);
    std::uniform_real_distribution<float> current_battery(CURRENT_LBND, CURRENT_UBND);

    battery->set_voltage(voltage_battery(g_mt));
    battery->set_current(current_battery(g_mt));
}

/**
 * @brief generate random wind sensors data
 *
 * @param wind_data pointer to generated wind sensors data
 */
void genRandWindData(Sensors::Wind * wind_data)
{
    std::uniform_real_distribution<float> speed_wind(SPEED_LBND, SPEED_UBND);
    std::uniform_int_distribution<int>    direction_wind(DIRECTION_LBND, DIRECTION_UBND);

    wind_data->set_speed(speed_wind(g_mt));
    wind_data->set_direction(direction_wind(g_mt));
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

    // gps
    sensors.set_allocated_gps(genRandGpsData());

    // ais ships, TODO(): Polaris should be included as one of the AIS ships
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        genRandAisData(sensors.add_ais_ships());
    }

    // generic sensors
    for (int i = 0; i < NUM_GENERIC_SENSORS; i++) {
        genRandGenericSensorData(sensors.add_data_sensors());
    }

    // batteries
    for (int i = 0; i < NUM_BATTERIES; i++) {
        genRandBatteriesData(sensors.add_batteries());
    }

    // wind sensors
    for (int i = 0; i < NUM_WIND_SENSORS; i++) {
        genRandWindData(sensors.add_wind_sensors());
    }

    // path waypoints
    for (int i = 0; i < NUM_PATH_WAYPOINTS; i++) {
        genRandPathData(sensors.add_local_path_data());
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

    // ais ships
    // Array size checking done in dumpSensors
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        const Sensors::Ais & dumped_ais_ships = dumped_sensors.ais_ships(i);
        const Sensors::Ais & rand_ais_ships   = rand_sensors.ais_ships(i);
        EXPECT_EQ(dumped_ais_ships.id(), rand_ais_ships.id());
        EXPECT_FLOAT_EQ(dumped_ais_ships.latitude(), rand_ais_ships.latitude());
        EXPECT_FLOAT_EQ(dumped_ais_ships.longitude(), rand_ais_ships.longitude());
        EXPECT_FLOAT_EQ(dumped_ais_ships.speed(), rand_ais_ships.speed());
        EXPECT_FLOAT_EQ(dumped_ais_ships.heading(), rand_ais_ships.heading());
        EXPECT_FLOAT_EQ(dumped_ais_ships.rot(), rand_ais_ships.rot());
        EXPECT_FLOAT_EQ(dumped_ais_ships.width(), rand_ais_ships.width());
        EXPECT_FLOAT_EQ(dumped_ais_ships.length(), rand_ais_ships.length());
    }

    // generic sensors
    for (int i = 0; i < NUM_GENERIC_SENSORS; i++) {
        const Sensors::Generic & dumped_data_sensors = dumped_sensors.data_sensors(i);
        const Sensors::Generic & rand_data_sensors   = rand_sensors.data_sensors(i);
        EXPECT_EQ(dumped_data_sensors.id(), rand_data_sensors.id());
        EXPECT_EQ(dumped_data_sensors.data(), rand_data_sensors.data());
    }

    // batteries
    for (int i = 0; i < NUM_BATTERIES; i++) {
        const Sensors::Battery & dumped_batteries = dumped_sensors.batteries(i);
        const Sensors::Battery & rand_batteries   = rand_sensors.batteries(i);
        EXPECT_EQ(dumped_batteries.voltage(), rand_batteries.voltage());
        EXPECT_EQ(dumped_batteries.current(), rand_batteries.current());
    }

    // wind sensors
    for (int i = 0; i < NUM_WIND_SENSORS; i++) {
        const Sensors::Wind & dumped_wind_sensors = dumped_sensors.wind_sensors(i);
        const Sensors::Wind & rand_wind_sensors   = rand_sensors.wind_sensors(i);
        EXPECT_EQ(dumped_wind_sensors.speed(), rand_wind_sensors.speed());
        EXPECT_EQ(dumped_wind_sensors.direction(), rand_wind_sensors.direction());
    }

    // path waypoints
    for (int i = 0; i < NUM_PATH_WAYPOINTS; i++) {
        const Sensors::Path & dumped_path_waypoints = dumped_sensors.local_path_data(i);
        const Sensors::Path & rand_path_waypoints   = rand_sensors.local_path_data(i);
        EXPECT_EQ(dumped_path_waypoints.latitude(), rand_path_waypoints.latitude());
        EXPECT_EQ(dumped_path_waypoints.longitude(), rand_path_waypoints.longitude());
    }

    // generic sensors
    for (int i = 0; i < NUM_GENERIC_SENSORS; i++) {
        const Sensors::Generic & dumped_data_sensors = dumped_sensors.data_sensors(i);
        const Sensors::Generic & rand_data_sensors   = rand_sensors.data_sensors(i);
        EXPECT_EQ(dumped_data_sensors.id(), rand_data_sensors.id());
        EXPECT_EQ(dumped_data_sensors.data(), rand_data_sensors.data());
    }

    // batteries
    for (int i = 0; i < NUM_BATTERIES; i++) {
        const Sensors::Battery & dumped_batteries = dumped_sensors.batteries(i);
        const Sensors::Battery & rand_batteries   = rand_sensors.batteries(i);
        EXPECT_EQ(dumped_batteries.voltage(), rand_batteries.voltage());
        EXPECT_EQ(dumped_batteries.current(), rand_batteries.current());
    }

    // wind sensors
    for (int i = 0; i < NUM_WIND_SENSORS; i++) {
        const Sensors::Wind & dumped_wind_sensors = dumped_sensors.wind_sensors(i);
        const Sensors::Wind & rand_wind_sensors   = rand_sensors.wind_sensors(i);
        EXPECT_EQ(dumped_wind_sensors.speed(), rand_wind_sensors.speed());
        EXPECT_EQ(dumped_wind_sensors.direction(), rand_wind_sensors.direction());
    }
}
