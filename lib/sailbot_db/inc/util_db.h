#pragma once

#include <random>
#include <span>

#include "sailbot_db.h"
#include "sensors.pb.h"

class UtilDB : public SailbotDB
{
public:
    static constexpr int NUM_AIS_SHIPS       = 15;  // arbitrary number
    static constexpr int NUM_GENERIC_SENSORS = 5;   // arbitrary number
    static constexpr int NUM_PATH_WAYPOINTS  = 5;   // arbitrary number

    /**
     * @brief Construct a UtilDB. Private because there should only ever be one instance, which should be
     *        inialized using the initUtilDB() function
     *
     * @param db_name
     * @param mongodb_conn_str
     * @param rng
     */
    UtilDB(const std::string & db_name, const std::string & mongodb_conn_str, std::shared_ptr<std::mt19937> rng);

    /**
     * @brief Initialize a static instance of the UtilDB
     *
     * @param db_name
     * @param mongodb_conn_str
     * @param rng
     * @return shared pointer to the static instance
     */
    static std::shared_ptr<UtilDB> initUtilDB(
      const std::string & db_name, const std::string & mongodb_conn_str, std::shared_ptr<std::mt19937> rng);

    /**
     * @brief Delete all documents in all collections
     */
    void cleanDB();

    /**
    * @brief Generate random data for all sensors
    *
    * @return Sensors object
    */
    Polaris::Sensors genRandSensors();

    /**
    * @brief Generate random sensors and Iridium msg info
    *
    * @return std::pair<Sensors, SailbotDB::RcvdMsgInfo>
    */
    std::pair<Polaris::Sensors, SailbotDB::RcvdMsgInfo> genRandData();

    /**
     * @brief Retrieve all sensors from the database sorted by timestamp
     *
     * @param num_docs expected number of documents for each collection, default 1
     *
     * @return Vector of sensors objects: gps, ais, generic, batteries, wind, local path
     * @return Vector of timestamps
     *         both vectors will be num_docs in size
     */
    std::pair<std::vector<Polaris::Sensors>, std::vector<std::string>> dumpSensors(size_t num_docs = 1);

    /**
    * @brief Query the database and check that the sensor and message are correct
    *
    * @param expected_sensors
    * @param expected_msg_info
    */
    void verifyDBWrite(
      std::span<Polaris::Sensors> expected_sensors, std::span<SailbotDB::RcvdMsgInfo> expected_msg_info);

private:
    std::shared_ptr<std::mt19937> rng_;

    /**
    * @brief generate random GPS data
    *
    * @param gps_data pointer to generated gps_data
    */
    void genRandGpsData(Polaris::Sensors::Gps & gps_data);

    /**
    * @brief generate random ais ships data
    *
    * @param ais_ship pointer to generated ais data
    */
    void genRandAisData(Polaris::Sensors::Ais & ais_ship);

    /**
    * @brief generate random generic sensor data
    *
    * @return pointer to generated generic sensor data
    */
    void genRandGenericSensorData(Polaris::Sensors::Generic & generic_sensor);

    /**
    * @brief generate random battery data
    *
    * @return pointer to generated battery data
    */
    void genRandBatteryData(Polaris::Sensors::Battery & battery);

    /**
    * @brief generate random wind sensors data
    *
    * @return pointer to generated wind sensors data
    */
    void genRandWindData(Polaris::Sensors::Wind & wind_data);

    /**
    * @brief generate random path data
    *
    * @return pointer to generated path data
    */
    void genRandPathData(Polaris::Sensors::Path & path_data);
};
