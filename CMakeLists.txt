cmake_minimum_required(VERSION 3.10)

set(LLVM_VERSION "10")

set(CMAKE_CXX_COMPILER "/usr/bin/clang++-${LLVM_VERSION}")
set(CMAKE_CXX_CLANG_TIDY "clang-tidy-${LLVM_VERSION}")
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -g -std=c++14 -Wall -Wextra -O0 \
    -fprofile-arcs -ftest-coverage -fprofile-instr-generate -fcoverage-mapping")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(network_systems)

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Boost
find_package(Boost 1.80.0 COMPONENTS REQUIRED
    # Insert desired libraries below (see /usr/include/boost/ for available options)
    serialization
)

# Protobuf
find_package(Protobuf REQUIRED)

# Add src directories
add_subdirectory(lib)
add_subdirectory(projects)

ament_package()
