#include "remote_transceiver.h"

#include <iostream>

#include "sailbot_db.h"

#ifdef NDEBUG
// Cannot have main function defined with Googletest enabled
// Cannot have multiple SailbotDB instances defined ANYWHERE
int main()
{
    SailbotDB sailbot_db("admin");
    if (!sailbot_db.testConnection()) {
        std::cout << "Failed to connect to database, exiting" << std::endl;
        return -1;
    }
    return 0;
}
#endif
