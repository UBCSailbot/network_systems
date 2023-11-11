#include <iostream>

#include "remote_transceiver.h"
#include "sailbot_db.h"

int main()
{
    // TODO(): Run parameters not clearly defined yet.
    // will need to switch "admin" with the relevant db
    SailbotDB sailbot_db("admin", MONGODB_CONN_STR);
    if (!sailbot_db.testConnection()) {
        std::cout << "Failed to connect to database, exiting" << std::endl;
        return -1;
    }

    // TODO(): Need to distinguish between test and deployment address + port
    bio::ip::address addr = bio::ip::make_address(TESTING_HOST);
    const uint32_t   port = TESTING_PORT;

    bio::io_context io;
    tcp::acceptor   acceptor{io, {addr, port}};
    tcp::socket     socket{io};

    remote_transceiver::HTTPServer::runServer(acceptor, socket, sailbot_db);

    io.run();

    return 0;
}
