#include "remote_transceiver.h"

#include <iostream>

#include "sailbot_db.h"

int main()
{
    // TODO(): Run parameters not clearly defined yet.
    // will need to switch "admin" with the relevant db and
    // make the connection string an input parameter to differentiate
    // between local testing and deployment at runtime
    SailbotDB sailbot_db("admin", "mongodb://localhost:27017");
    if (!sailbot_db.testConnection()) {
        std::cout << "Failed to connect to database, exiting" << std::endl;
        return -1;
    }
    return 0;
}
