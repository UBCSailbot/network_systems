#pragma once

#include <google/protobuf/repeated_field.h>

#include "mongocxx/client.hpp"
#include "mongocxx/collection.hpp"
#include "mongocxx/instance.hpp"
#include "sensors.pb.h"

constexpr auto COLLECTION_AIS_SHIPS    = "ais_ships";
constexpr auto COLLECTION_BATTERIES    = "batteries";
constexpr auto COLLECTION_DATA_SENSORS = "data_sensors";
constexpr auto COLLECTION_GPS          = "gps";
constexpr auto COLLECTION_WIND_SENSORS = "wind_sensors";
constexpr auto COLLECTION_PATH         = "path";

template <typename T>
using ProtoList = google::protobuf::RepeatedPtrField<T>;
using DocVal    = bsoncxx::document::value;

/**
 * Class that encapsulates a Sailbot MongoDB database
 *
 */
class SailbotDB
{
public:
    /**
    * @brief Construct a new SailbotDB object
    *
    * @param db_name          name of desired database
    * @param mongodb_conn_str URL for mongodb database (ex. mongodb://localhost:27017)
    */
    explicit SailbotDB(const std::string & db_name, const std::string & mongodb_conn_str);

    /**
     * @brief Format and print a document in the DB
     *
     * @param doc document value
     */
    static void printDoc(const DocVal & doc);

    /**
     * @brief Ping the connected database to see if the connection succeeded
     *
     * @return true  if ping is successful
     * @return false if ping fails
     */
    bool testConnection();

    /**
     * @brief Write sensor data to the database
     *
     * @param sensors_pb Protobuf Sensors object
     * @return true  if successful
     * @return false on failure
     */
    bool storeSensors(const Polaris::Sensors & sensors_pb);

protected:
    mongocxx::database db_;  // MongoDB database this object is attached to

private:
    mongocxx::client   client_;  // Connected MongoDB client (must be present)
    mongocxx::instance inst_;    // MongoDB instance (must be present - there can only ever be one)

    /**
     * @brief Write GPS data to the database
     *
     * @param gps_pb Protobuf GPS object
     * @return true  if successful
     * @return false on failure
     */
    bool storeGps(const Polaris::Sensors::Gps & gps_pb);

    /**
     * @brief Write AIS data to the database
     *
     * @param ais_ships_pb Protobuf list of AIS objects, where the size of the list is the number of ships
     * @return true  if successful
     * @return false on failure
     */
    bool storeAis(const ProtoList<Polaris::Sensors::Ais> & ais_ships_pb);

    /**
     * @brief Write generic sensor data to the database
     *
     * @param generic_pb Protobuf list of generic sensor objects, where the size of the list is the number of sensors
     * @return true  if successful
     * @return false on failure
     */
    bool storeGenericSensor(const ProtoList<Polaris::Sensors::Generic> & generic_pb);

    /**
     * @brief Write batteries data to the database
     *
     * @param battery_pb Protobuf list of battery objects, where the size of the list is the number of batteries
     * @return true  if successful
     * @return false on failure
     */
    bool storeBatteries(const ProtoList<Polaris::Sensors::Battery> & battery_pb);

    /**
     * @brief Write wind sensor data to the database
     *
     * @param generic_pb Protobuf list of wind sensor objects, where the size of the list is the number of wind sensors
     * @return true  if successful
     * @return false on failure
     */
    bool storeWindSensor(const ProtoList<Polaris::Sensors::Wind> & wind_pb);

    /**
     * @brief Write path sensor data to the database
     *
     * @param generic_pb Protobuf list of path sensor objects, where the size of the list is the number of path sensors
     * @return true  if successful
     * @return false on failure
     */
    bool storePathSensor(const ProtoList<Polaris::Sensors::Path> & path_pb);
};
