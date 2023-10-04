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
    * @param db_name name of desired database
    */
    explicit SailbotDB(const std::string & db_name);

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

    /**
     * @brief Write AIS data to the database
     *
     * @param ais_ships_pb Protobuf list of AIS objects, where the size of the list is the number of ships
     * @return true  if successful
     * @return false on failure
     */
    bool storeAis(const ProtoList<Placeholder::Sensors::Ais> & ais_ships_pb);
};
