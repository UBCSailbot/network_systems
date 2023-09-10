#include "sailbot_db.h"

#include <iostream>

#include "bsoncxx/builder/basic/document.hpp"
#include "mongocxx/client.hpp"
#include "mongocxx/collection.hpp"
#include "mongocxx/instance.hpp"
#include "sensors.pb.h"

using bsoncxx::builder::basic::kvp;
using bsoncxx::builder::basic::make_array;
using bsoncxx::builder::basic::make_document;
using DocVal = bsoncxx::document::value;

// PUBLIC

SailbotDB::SailbotDB(const std::string & db_name)
{
    // inst_ implicitly initialized and will throw an exception if we try to explicitly initialize it like below
    // inst_             = {};
    mongocxx::uri uri = mongocxx::uri{MONGO_DB_CONNECTION_STRING};
    client_           = {uri};
    db_               = client_[db_name];
}

bool SailbotDB::testConnection()
{
    try {
        // Ping the database.
        const DocVal ping_cmd = make_document(bsoncxx::builder::basic::kvp("ping", 1));
        db_.run_command(ping_cmd.view());
        return true;
    } catch (const std::exception & e) {
        std::cout << "Exception: " << e.what() << std::endl;
        return false;
    }
}

bool SailbotDB::storeSensors(const Placeholder::Sensors & sensors_pb) { return storeGps(sensors_pb.gps()); }

// END PUBLIC

// PRIVATE

bool SailbotDB::storeGps(const Placeholder::Sensors::Gps & gps_pb)
{
    mongocxx::collection gps_coll = db_[COLLECTION_GPS];
    const DocVal         gps_doc  = make_document(
               kvp("lat", gps_pb.lat()), kvp("lon", gps_pb.lon()), kvp("speed", gps_pb.speed()),
               kvp("heading", gps_pb.heading()));
    return static_cast<bool>(gps_coll.insert_one(bsoncxx::document::view_or_value(gps_doc)));
}

// END PRIVATE
