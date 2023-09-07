#include <string>

#include "bsoncxx/builder/basic/document.hpp"
#include "mongocxx/client.hpp"
#include "mongocxx/collection.hpp"
#include "mongocxx/instance.hpp"
#include "sensors.pb.h"

constexpr auto MONGO_DB_CONNECTION_STRING = "mongodb://localhost:27017";
constexpr auto COLLECTION_GPS             = "gps";
constexpr auto COLLECTION_AIS             = "ais_ships";
constexpr auto COLLECTION_DATA_SENSORS    = "data_sensors";
constexpr auto COLLECTION_WIND            = "wind_sensors";

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

    void storeSensors(const Placeholder::Sensors & sensors_pb);

protected:
    mongocxx::instance inst_;
    mongocxx::client   client_;
    mongocxx::database db_;

private:
    void storeGps(const Placeholder::Sensors::Gps & gps_pb);
};
