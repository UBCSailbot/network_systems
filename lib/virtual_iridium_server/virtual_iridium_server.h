#pragma once

#include <sys/wait.h>

#include <boost/process.hpp>
#include <chrono>
#include <thread>

namespace bp = boost::process;

/**
 * Wrapper class to run the virtual iridium server in a background process
 * See network_systems/scripts/README.md and run_virtual_iridium.sh for more information
 */
class VirtualIridiumServer
{
public:
    /**
    * @brief Spawn a virtual iridium process in the background
    *
    */
    VirtualIridiumServer()
    {
        server_proc_ = bp::child(RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH);
        std::this_thread::sleep_for(std::chrono::seconds(1));  // Need to wait for test ports to generate
    };

    /**
     * @brief Calls shutdown()
     *
     */
    ~VirtualIridiumServer() { shutdown(); }

    /**
     * @brief Kills the spawned virtual iridium process and its sub-processes
     *
     */
    void shutdown()
    {
        std::cout << server_proc_.id() << std::endl;
        // std::cout << "SHUTDOWN" << std::endl;
        std::cout << kill(server_proc_.id(), SIGINT) << std::endl;
        for (int i = 0; i < 15; i++) {  //NOLINT
            pid_t endId = waitpid(server_proc_.id(), nullptr, WNOHANG);
            std::cout << endId << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

private:
    bp::child server_proc_;  // Handle to the spawned virtual iridium process
};
