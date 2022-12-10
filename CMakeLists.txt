cmake_minimum_required(VERSION 3.10)
project(network_systems)

# Options and constants
option(STATIC_ANALYSIS "Enable clang-tidy checks" ON)
set(LLVM_VERSION "10")

# Configure the compiler
set(CMAKE_CXX_COMPILER "/usr/bin/clang++-${LLVM_VERSION}")
if(STATIC_ANALYSIS)
    set(CMAKE_CXX_CLANG_TIDY "clang-tidy-${LLVM_VERSION}")
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Configure build flags
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_DEBUG} -fprofile-arcs \
        -ftest-coverage -fprofile-instr-generate -fcoverage-mapping")
    add_definitions(-DDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endif()
# Enable all warnings as errors except unused parameter because auto-generated protobuf files trigger it,
# and clang-tidy can cover it without false-flagging the protobuf files.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter -pthread")
message("Building Network Systems with build type '${CMAKE_BUILD_TYPE}' "
        "and flags: '${CMAKE_CXX_FLAGS}'")

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Boost
find_package(Boost 1.74.0 COMPONENTS REQUIRED
    # Insert desired libraries below (see /usr/include/boost/ for available options)
    serialization
)

# Protobuf
find_package(Protobuf REQUIRED)

# Googletest (installed by sailbot_workspace Docker config)
set(GTEST_LINK_LIBS "/usr/local/lib/libgtest.a" "/usr/local/lib/libgtest_main.a")

# Add src directories
add_subdirectory(lib)
add_subdirectory(projects)

# Install launch files
install(DIRECTORY launch DESTINATION share/network_systems)

ament_package()
