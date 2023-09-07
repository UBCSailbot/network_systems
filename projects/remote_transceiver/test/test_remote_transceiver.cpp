#include "bsoncxx/builder/basic/document.hpp"
#include "gtest/gtest.h"
#include "mongocxx/collection.hpp"
#include "remote_transceiver.h"
#include "sailbot_db.h"

using bsoncxx::builder::basic::make_document;

class TestDB : public SailbotDB
{
public:
    TestDB() : SailbotDB("test") {}
    void clean_db()
    {
        mongocxx::collection gps_coll = db_[COLLECTION_GPS];
        gps_coll.delete_many(make_document());
    }
};

TestDB g_test_db = {};

class TestRemoteTransceiver : public ::testing::Test
{
protected:
    TestRemoteTransceiver() { g_test_db = TestDB(); }
    ~TestRemoteTransceiver() override { g_test_db.clean_db(); }
};
