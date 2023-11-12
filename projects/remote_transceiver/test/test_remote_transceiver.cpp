#include <gtest/gtest.h>

#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/address.hpp>
#include <boost/beast/http/status.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <chrono>
#include <cstdint>
#include <mongocxx/client.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/cursor.hpp>
#include <mongocxx/database.hpp>
#include <random>
#include <string>
#include <thread>

#include "cmn_hdrs/shared_constants.h"
#include "remote_transceiver.h"
#include "sailbot_db.h"
#include "sensors.pb.h"

constexpr int NUM_AIS_SHIPS       = 15;  // arbitrary number
constexpr int NUM_GENERIC_SENSORS = 5;   // arbitrary number
constexpr int NUM_PATH_WAYPOINTS  = 5;   // arbitrary number

using Polaris::Sensors;
using remote_transceiver::HTTPServer;
using remote_transceiver::TESTING_HOST;
using remote_transceiver::TESTING_PORT;
namespace http_client = remote_transceiver::http_client;

//Child class of SailbotDB that includes additional database utility functions to help testing
class TestDB : public SailbotDB
{
public:
    static constexpr auto TEST_DB = "test";
    TestDB() : SailbotDB(TEST_DB, MONGODB_CONN_STR) {}

    /**
     * @brief Delete all documents in all collections
     */
    void cleanDB()
    {
        mongocxx::pool::entry entry = pool_->acquire();
        mongocxx::database    db    = (*entry)[db_name_];

        mongocxx::collection gps_coll        = db[COLLECTION_GPS];
        mongocxx::collection ais_coll        = db[COLLECTION_AIS_SHIPS];
        mongocxx::collection generic_coll    = db[COLLECTION_DATA_SENSORS];
        mongocxx::collection batteries_coll  = db[COLLECTION_BATTERIES];
        mongocxx::collection wind_coll       = db[COLLECTION_WIND_SENSORS];
        mongocxx::collection local_path_coll = db[COLLECTION_LOCAL_PATH];

        gps_coll.delete_many(bsoncxx::builder::basic::make_document());
        ais_coll.delete_many(bsoncxx::builder::basic::make_document());
        generic_coll.delete_many(bsoncxx::builder::basic::make_document());
        batteries_coll.delete_many(bsoncxx::builder::basic::make_document());
        wind_coll.delete_many(bsoncxx::builder::basic::make_document());
        local_path_coll.delete_many(bsoncxx::builder::basic::make_document());
    }

    /**
     * @return Sensors objects: gps, ais, generic, batteries, wind, local path
     */
    std::pair<Sensors, std::string> dumpSensors()
    {
        Sensors               sensors;
        std::string           timestamp;
        mongocxx::pool::entry entry = pool_->acquire();
        mongocxx::database    db    = (*entry)[db_name_];

        // gps
        mongocxx::collection gps_coll = db[COLLECTION_GPS];
        mongocxx::cursor     gps_docs = gps_coll.find({});
        EXPECT_EQ(gps_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        Sensors::Gps *          gps     = sensors.mutable_gps();
        bsoncxx::document::view gps_doc = *(gps_docs.begin());
        gps->set_latitude(static_cast<float>(gps_doc["latitude"].get_double().value));
        gps->set_longitude(static_cast<float>(gps_doc["longitude"].get_double().value));
        gps->set_speed(static_cast<float>(gps_doc["speed"].get_double().value));
        gps->set_heading(static_cast<float>(gps_doc["heading"].get_double().value));
        timestamp = gps_doc["timestamp"].get_utf8().value.to_string();

        // ais ships
        mongocxx::collection ais_coll = db[COLLECTION_AIS_SHIPS];
        mongocxx::cursor     ais_docs = ais_coll.find({});
        EXPECT_EQ(ais_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view ais_ships_doc = *(ais_docs.begin());
        for (bsoncxx::array::element ais_ships_doc : ais_ships_doc["ships"].get_array().value) {
            Sensors::Ais * ais_ship = sensors.add_ais_ships();
            ais_ship->set_id(static_cast<uint32_t>(ais_ships_doc["id"].get_int64().value));
            ais_ship->set_latitude(static_cast<float>(ais_ships_doc["latitude"].get_double().value));
            ais_ship->set_longitude(static_cast<float>(ais_ships_doc["longitude"].get_double().value));
            ais_ship->set_sog(static_cast<float>(ais_ships_doc["sog"].get_double().value));
            ais_ship->set_cog(static_cast<float>(ais_ships_doc["cog"].get_double().value));
            ais_ship->set_rot(static_cast<float>(ais_ships_doc["rot"].get_double().value));
            ais_ship->set_width(static_cast<float>(ais_ships_doc["width"].get_double().value));
            ais_ship->set_length(static_cast<float>(ais_ships_doc["length"].get_double().value));
        }
        EXPECT_EQ(sensors.ais_ships().size(), NUM_AIS_SHIPS) << "Size mismatch when reading AIS ships from DB";
        EXPECT_EQ(ais_ships_doc["timestamp"].get_utf8().value.to_string(), timestamp)
          << "Timestamps must all be the same";

        // generic sensor
        mongocxx::collection generic_coll        = db[COLLECTION_DATA_SENSORS];
        mongocxx::cursor     generic_sensor_docs = generic_coll.find({});
        EXPECT_EQ(generic_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view generic_doc = *(generic_sensor_docs.begin());
        for (bsoncxx::array::element generic_doc : generic_doc["genericSensors"].get_array().value) {
            Sensors::Generic * generic = sensors.add_data_sensors();
            generic->set_id(static_cast<uint32_t>(generic_doc["id"].get_int64().value));
            generic->set_data(static_cast<uint64_t>(generic_doc["data"].get_int64().value));
        }
        EXPECT_EQ(generic_doc["timestamp"].get_utf8().value.to_string(), timestamp)
          << "Timestamps must all be the same";

        // battery
        mongocxx::collection batteries_coll      = db[COLLECTION_BATTERIES];
        mongocxx::cursor     batteries_data_docs = batteries_coll.find({});
        EXPECT_EQ(batteries_coll.count_documents({}), 1)
          << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view batteries_doc = *(batteries_data_docs.begin());
        for (bsoncxx::array::element batteries_doc : batteries_doc["batteries"].get_array().value) {
            Sensors::Battery * battery = sensors.add_batteries();
            battery->set_voltage(static_cast<float>(batteries_doc["voltage"].get_double().value));
            battery->set_current(static_cast<float>(batteries_doc["current"].get_double().value));
        }
        EXPECT_EQ(sensors.batteries().size(), NUM_BATTERIES) << "Size mismatch when reading batteries from DB";
        EXPECT_EQ(batteries_doc["timestamp"].get_utf8().value.to_string(), timestamp)
          << "Timestamps must all be the same";

        // wind sensor
        mongocxx::collection wind_coll         = db[COLLECTION_WIND_SENSORS];
        mongocxx::cursor     wind_sensors_docs = wind_coll.find({});
        EXPECT_EQ(wind_coll.count_documents({}), 1) << "Error: TestDB should only have one document per collection";
        bsoncxx::document::view wind_doc = *(wind_sensors_docs.begin());
        for (bsoncxx::array::element wind_doc : wind_doc["windSensors"].get_array().value) {
            Sensors::Wind * wind = sensors.add_wind_sensors();
            wind->set_speed(static_cast<float>(wind_doc["speed"].get_double().value));
            wind->set_direction(static_cast<int16_t>(wind_doc["direction"].get_int32().value));
        }
        EXPECT_EQ(sensors.wind_sensors().size(), NUM_WIND_SENSORS) << "Size mismatch when reading batteries from DB";
        EXPECT_EQ(wind_doc["timestamp"].get_utf8().value.to_string(), timestamp) << "Timestamps must all be the same";

        // local path
        mongocxx::collection path_coll       = db[COLLECTION_LOCAL_PATH];
        mongocxx::cursor     local_path_docs = path_coll.find({});
        EXPECT_EQ(path_coll.count_documents({}), 1) << "Error: Test DB should only have one document per collection";
        bsoncxx::document::view path_doc = *(local_path_docs.begin());
        for (bsoncxx::array::element path_doc : path_doc["waypoints"].get_array().value) {
            Polaris::Waypoint * path = sensors.add_local_path_data();
            path->set_latitude(static_cast<float>(path_doc["latitude"].get_double().value));
            path->set_longitude(static_cast<float>(path_doc["longitude"].get_double().value));
        }
        EXPECT_EQ(sensors.local_path_data().size(), NUM_PATH_WAYPOINTS)
          << "Size mismatch when reading path waypoints from DB";
        EXPECT_EQ(path_doc["timestamp"].get_utf8().value.to_string(), timestamp) << "Timestamps must all be the same";

        return {sensors, timestamp};
    }
};

static TestDB             g_test_db   = TestDB();              // initialize the TestDB instance
static std::random_device g_rd        = std::random_device();  // random number sampler
static uint32_t           g_rand_seed = g_rd();                // seed used for random number generation
static std::mt19937       g_mt(g_rand_seed);                   // initialize random number generator with seed

class TestSailbotDB : public ::testing::Test
{
protected:
    TestSailbotDB() { g_test_db.cleanDB(); }
    ~TestSailbotDB() override {}
};

/**
 * @brief generate random GPS data
 *
 * @param gps_data pointer to generated gps_data
 */
void * genRandGpsData(Sensors::Gps * gps_data)
{
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
    ais_ship->set_sog(speed_dist(g_mt));
    ais_ship->set_cog(heading_dist(g_mt));
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
void genRandPathData(Polaris::Waypoint * path_data)
{
    std::uniform_real_distribution<float> latitude_path(LAT_LBND, LAT_UBND);
    std::uniform_real_distribution<float> longitude_path(LON_LBND, LON_UBND);

    path_data->set_latitude(latitude_path(g_mt));
    path_data->set_longitude(longitude_path(g_mt));
}

/**
 * @brief Generate random data for all sensors
 *
 * @return Sensors object
 */
Sensors genRandSensors()
{
    Sensors sensors;

    // gps
    genRandGpsData(sensors.mutable_gps());

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
}

/**
 * @brief Generate random sensors and Iridium msg info
 *
 * @return std::pair<Sensors, SailbotDB::RcvdMsgInfo>
 */
std::pair<Sensors, SailbotDB::RcvdMsgInfo> genRandData()
{
    Sensors                rand_sensors = genRandSensors();
    SailbotDB::RcvdMsgInfo randInfo{
      .lat_       = 0,                           // Not processed yet, so just set to 0
      .lon_       = 0,                           // Not processed yet, so just set to 0
      .cep_       = 0,                           // Not processed yet, so just set to 0
      .timestamp_ = std::to_string(g_rand_seed)  // Any random string works for testing
    };
    return {rand_sensors, randInfo};
}

/**
 * @brief Query the database and check that the sensor and message are correct
 *
 * @param expected_sensors
 * @param expected_msg_info
 */
void verifyDBWrite(Sensors expected_sensors, SailbotDB::RcvdMsgInfo expected_msg_info)
{
    auto [dumped_sensors, dumped_timestamp] = g_test_db.dumpSensors();

    EXPECT_EQ(dumped_timestamp, expected_msg_info.timestamp_);

    // gps
    EXPECT_FLOAT_EQ(dumped_sensors.gps().latitude(), expected_sensors.gps().latitude());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().longitude(), expected_sensors.gps().longitude());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().speed(), expected_sensors.gps().speed());
    EXPECT_FLOAT_EQ(dumped_sensors.gps().heading(), expected_sensors.gps().heading());

    // ais ships
    // Array size checking done in dumpSensors
    for (int i = 0; i < NUM_AIS_SHIPS; i++) {
        const Sensors::Ais & dumped_ais_ship   = dumped_sensors.ais_ships(i);
        const Sensors::Ais & expected_ais_ship = expected_sensors.ais_ships(i);
        EXPECT_EQ(dumped_ais_ship.id(), expected_ais_ship.id());
        EXPECT_FLOAT_EQ(dumped_ais_ship.latitude(), expected_ais_ship.latitude());
        EXPECT_FLOAT_EQ(dumped_ais_ship.longitude(), expected_ais_ship.longitude());
        EXPECT_FLOAT_EQ(dumped_ais_ship.sog(), expected_ais_ship.sog());
        EXPECT_FLOAT_EQ(dumped_ais_ship.cog(), expected_ais_ship.cog());
        EXPECT_FLOAT_EQ(dumped_ais_ship.rot(), expected_ais_ship.rot());
        EXPECT_FLOAT_EQ(dumped_ais_ship.width(), expected_ais_ship.width());
        EXPECT_FLOAT_EQ(dumped_ais_ship.length(), expected_ais_ship.length());
    }

    // generic sensors
    for (int i = 0; i < NUM_GENERIC_SENSORS; i++) {
        const Sensors::Generic & dumped_data_sensor   = dumped_sensors.data_sensors(i);
        const Sensors::Generic & expected_data_sensor = expected_sensors.data_sensors(i);
        EXPECT_EQ(dumped_data_sensor.id(), expected_data_sensor.id());
        EXPECT_EQ(dumped_data_sensor.data(), expected_data_sensor.data());
    }

    // batteries
    for (int i = 0; i < NUM_BATTERIES; i++) {
        const Sensors::Battery & dumped_battery   = dumped_sensors.batteries(i);
        const Sensors::Battery & expected_battery = expected_sensors.batteries(i);
        EXPECT_EQ(dumped_battery.voltage(), expected_battery.voltage());
        EXPECT_EQ(dumped_battery.current(), expected_battery.current());
    }

    // wind sensors
    for (int i = 0; i < NUM_WIND_SENSORS; i++) {
        const Sensors::Wind & dumped_wind_sensor   = dumped_sensors.wind_sensors(i);
        const Sensors::Wind & expected_wind_sensor = expected_sensors.wind_sensors(i);
        EXPECT_EQ(dumped_wind_sensor.speed(), expected_wind_sensor.speed());
        EXPECT_EQ(dumped_wind_sensor.direction(), expected_wind_sensor.direction());
    }

    // path waypoints
    for (int i = 0; i < NUM_PATH_WAYPOINTS; i++) {
        const Polaris::Waypoint & dumped_path_waypoint   = dumped_sensors.local_path_data(i);
        const Polaris::Waypoint & expected_path_waypoint = expected_sensors.local_path_data(i);
        EXPECT_EQ(dumped_path_waypoint.latitude(), expected_path_waypoint.latitude());
        EXPECT_EQ(dumped_path_waypoint.longitude(), expected_path_waypoint.longitude());
    }
}

/**
 * @brief Check that MongoDB is running
 */
TEST_F(TestSailbotDB, TestConnection)
{
    ASSERT_TRUE(g_test_db.testConnection()) << "MongoDB not running - remember to connect!";
}

/**
 * @brief Write random sensor data to the TestDB - read and verify said data
 */
TEST_F(TestSailbotDB, TestStoreSensors)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [rand_sensors, randInfo] = genRandData();
    ASSERT_TRUE(g_test_db.storeNewSensors(rand_sensors, randInfo));

    verifyDBWrite(rand_sensors, randInfo);
}

class TestHTTP : public ::testing::Test
{
protected:
    // Need to wait after receiving an HTTP response from the server
    static constexpr auto WAIT_AFTER_RES = std::chrono::milliseconds(20);

    // Network objects that are shared amongst all HTTP test suites
    static bio::io_context * io_;
    static tcp::acceptor *   acceptor_;
    static tcp::socket *     socket_;
    static std::thread *     io_thread_;
    static SailbotDB *       server_db_;

    static void SetUpTestSuite()
    {
        io_        = new bio::io_context;
        acceptor_  = new tcp::acceptor{*io_, {bio::ip::make_address(TESTING_HOST), TESTING_PORT}};
        socket_    = new tcp::socket{*io_};
        server_db_ = new TestDB();

        HTTPServer::runServer(*acceptor_, *socket_, *server_db_);

        io_thread_ = new std::thread([]() { io_->run(); });
    }

    static void TearDownTestSuite()
    {
        io_->stop();
        io_thread_->join();

        delete io_;
        delete acceptor_;
        delete socket_;
        delete io_thread_;
        delete server_db_;
    }

    TestHTTP() { g_test_db.cleanDB(); }

    ~TestHTTP() override {}
};

// Initialize static objects
bio::io_context * TestHTTP::io_        = nullptr;
tcp::acceptor *   TestHTTP::acceptor_  = nullptr;
tcp::socket *     TestHTTP::socket_    = nullptr;
std::thread *     TestHTTP::io_thread_ = nullptr;
SailbotDB *       TestHTTP::server_db_ = nullptr;

/**
 * @brief Test HTTP GET request sending and handling. Currently just retrieves a placeholder string.
 *
 */
TEST_F(TestHTTP, TestGet)
{
    auto [status, result] =
      http_client::get({TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::ROOT});
    EXPECT_EQ(status, http::status::ok);
    EXPECT_EQ(result, "PLACEHOLDER\r\n");
}

/**
 * @brief Create a formatted string that matches the body of POST requests from Iridium
 *        https://docs.rockblock.rock7.com/reference/receiving-mo-messages-via-http-webhook
 *
 * @param params Params structure
 * @return formatted request body
 */
std::string createPostBody(remote_transceiver::MOMsgParams::Params params)
{
    std::ostringstream s;
    s << "imei=" << params.imei_ << "&serial=" << params.serial_ << "&momsn=" << params.momsn_
      << "&transmit_time=" << params.transmit_time_ << "&iridium_latitude=" << params.lat_
      << "&iridium_longitude=" << params.lon_ << "&iridium_cep=" << params.cep_ << "&data=" << params.data_;
    return s.str();
}

/**
 * @brief Test that we can POST sensor data to the server
 *
 */
TEST_F(TestHTTP, TestPostSensors)
{
    SCOPED_TRACE("Seed: " + std::to_string(g_rand_seed));  // Print seed on any failure
    auto [rand_sensors, randInfo] = genRandData();

    std::string rand_sensors_str;
    ASSERT_TRUE(rand_sensors.SerializeToString(&rand_sensors_str));
    Polaris::Sensors test;
    test.ParseFromString(rand_sensors_str);
    // This query is comprised entirely of arbitrary values exccept for .data_
    std::string query = createPostBody(
      {.imei_          = 0,
       .serial_        = 0,
       .momsn_         = 1,
       .transmit_time_ = "",
       .lat_           = 0.0,
       .lon_           = 0.0,
       .cep_           = 1,
       .data_          = rand_sensors_str});
    http::status status = http_client::post(
      {TESTING_HOST, std::to_string(TESTING_PORT), remote_transceiver::targets::SENSORS},
      "application/x-www-form-urlencoded", query);

    EXPECT_EQ(status, http::status::ok);
    std::this_thread::sleep_for(WAIT_AFTER_RES);
    verifyDBWrite(rand_sensors, {.lat_ = 0.0, .lon_ = 0.0, .cep_ = 1, .timestamp_ = ""});
}
