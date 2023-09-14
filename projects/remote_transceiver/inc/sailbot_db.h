#pragma once

#include "bsoncxx/builder/basic/document.hpp"
#include "mongocxx/client.hpp"
#include "mongocxx/collection.hpp"
#include "mongocxx/instance.hpp"
#include "sensors.pb.h"

constexpr auto COLLECTION_GPS          = "gps";
constexpr auto COLLECTION_AIS          = "ais_ships";
constexpr auto COLLECTION_DATA_SENSORS = "data_sensors";
constexpr auto COLLECTION_WIND         = "wind_sensors";

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
    * @param db_name name of desired database
    */
    explicit SailbotDB(const std::string & db_name);

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
    bool storeSensors(const Placeholder::Sensors & sensors_pb);

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
    bool storeGps(const Placeholder::Sensors::Gps & gps_pb);
};
