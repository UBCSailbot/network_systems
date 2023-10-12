#include "sailbot_db.h"

#include <bsoncxx/builder/stream/helpers.hpp>
#include <bsoncxx/json.hpp>
#include <iostream>

#include "bsoncxx/builder/stream/array.hpp"
#include "bsoncxx/builder/stream/document.hpp"
#include "mongocxx/client.hpp"
#include "mongocxx/collection.hpp"
#include "mongocxx/instance.hpp"
#include "sensors.pb.h"

namespace bstream = bsoncxx::builder::stream;
using Polaris::Sensors;

// PUBLIC

SailbotDB::SailbotDB(const std::string & db_name, const std::string & mongodb_conn_str)
{
    // inst_ implicitly initialized and will throw an exception if we try to explicitly initialize it like below
    // inst_             = {};
    mongocxx::uri uri = mongocxx::uri{mongodb_conn_str};
    client_           = {uri};
    db_               = client_[db_name];
}

void SailbotDB::printDoc(const DocVal & doc) { std::cout << bsoncxx::to_json(doc.view()) << std::endl; }

bool SailbotDB::testConnection()
{
    try {
        // Ping the database.
        const DocVal ping_cmd = bstream::document{} << "ping" << 1 << bstream::finalize;
        db_.run_command(ping_cmd.view());
        return true;
    } catch (const std::exception & e) {
        std::cout << "Exception: " << e.what() << std::endl;
        return false;
    }
}

bool SailbotDB::storeSensors(const Sensors & sensors_pb)
{
    return storeGps(sensors_pb.gps()) && storeAis(sensors_pb.ais_ships());
}

// END PUBLIC

// PRIVATE

bool SailbotDB::storeGps(const Sensors::Gps & gps_pb)
{
    mongocxx::collection gps_coll = db_[COLLECTION_GPS];
    DocVal gps_doc = bstream::document{} << "latitude" << gps_pb.latitude() << "longitude" << gps_pb.longitude()
                                         << "speed" << gps_pb.speed() << "heading" << gps_pb.heading()
                                         << bstream::finalize;
    return static_cast<bool>(gps_coll.insert_one(gps_doc.view()));
}

bool SailbotDB::storeAis(const ProtoList<Sensors::Ais> & ais_ships_pb)
{
    mongocxx::collection ais_coll = db_[COLLECTION_AIS_SHIPS];
    bstream::document    doc_builder{};
    auto                 ais_ships_doc_arr = doc_builder << "ais_ships" << bstream::open_array;
    for (const Sensors::Ais & ais_ship : ais_ships_pb) {
        // The BSON spec does not allow unsigned integers (throws exception), so cast our uint32s to sint64s
        ais_ships_doc_arr = ais_ships_doc_arr << bstream::open_document << "id" << static_cast<int64_t>(ais_ship.id())
                                              << "latitude" << ais_ship.latitude() << "longitude"
                                              << ais_ship.longitude() << "speed" << ais_ship.speed() << "heading"
                                              << ais_ship.heading() << bstream::close_document;
    }
    DocVal ais_ships_doc = ais_ships_doc_arr << bstream::close_array << bstream::finalize;
    return static_cast<bool>(ais_coll.insert_one(ais_ships_doc.view()));
}

bool SailbotDB::storeGenericSensor(const ProtoList<Sensors::Generic> & generic_pb)
{
    mongocxx::collection generic_coll = db_[COLLECTION_DATA_SENSORS];
    bstream::document    doc_builder{};
    auto                 generic_doc_arr = doc_builder << "data_sensors" << bstream::open_array;
    for (const Sensors::Generic & generic : generic_pb) {
        generic_doc_arr = generic_doc_arr << bstream::open_document << "id" << static_cast<int16_t>(generic.id())
                                          << "data" << static_cast<int64_t>(generic.data()) << bstream::close_document;
    }
    DocVal generic_doc = generic_doc_arr << bstream::close_array << bstream::finalize;
    return static_cast<bool>(generic_coll.insert_one(generic_doc.view()));
}


bool SailbotDB::storeBatteries(const ProtoList<Sensors::Battery> & battery_pb)
{
    mongocxx::collection batteries_coll = db_[COLLECTION_BATTERIES];
    bstream::document    doc_builder{};
    auto                 batteries_doc_arr = doc_builder << "batteries" << bstream::open_array;
    for (const Sensors::Battery & battery : battery_pb){
        batteries_doc_arr = batteries_doc_arr << bstream::open_document << "voltage" << battery.voltage()
                                            << "current" << battery.current() << bstream::close_document;
    }
    DocVal batteries_doc = batteries_doc_arr << bstream::close_array << bstream::finalize;
    return static_cast<bool>(batteries_coll.insert_one(batteries_doc.view()));
}

bool SailbotDB::storeWindSensor(const ProtoList<Sensors::Wind> & wind_pb)
{
    mongocxx::collection wind_coll = db_[COLLECTION_WIND_SENSORS];
    bstream::document     doc_builder{};
    auto                  wind_doc_arr = doc_builder << "wind_sensors" << bstream::open_array;
    for (const Sensors::Wind & wind_sensor : wind_pb){
        wind_doc_arr = wind_doc_arr << bstream::open_document << "speed" << wind_sensor.speed() << "direction"
                                    << static_cast<int16_t>(wind_sensor.direction()) << bstream::close_document;
    }
    DocVal wind_doc = wind_doc_arr << bstream::close_array << bstream::finalize;
    return static_cast<bool>(wind_coll.insert_one(wind_doc.view()));
}



// END PRIVATE
