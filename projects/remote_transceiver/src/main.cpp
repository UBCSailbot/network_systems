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

    try {
        // TODO(): Need to distinguish between test and deployment address + port
        bio::io_context  io;
        bio::ip::address addr = bio::ip::make_address(remote_transceiver::TESTING_HOST);
        const uint32_t   port = remote_transceiver::TESTING_PORT;
        tcp::acceptor    acceptor{io, {addr, port}};
        tcp::socket      socket{io};

        remote_transceiver::HTTPServer::runServer(acceptor, socket, sailbot_db);

        io.run();
    } catch (std::exception & e) {
        std::cerr << "Failed to run remote transceiver" << std::endl;
        std::cerr << e.what() << std::endl;
        return -1;
    }

    return 0;
}
