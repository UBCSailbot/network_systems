set(module can_transceiver)

set(link_libs
)

set(inc_dirs
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_frame_parser.cpp
)

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver_ros_intf.cpp
)
make_ros_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}")

# # can_transceiver.cpp (Publishes to /SimulatorFeedback)
# cmake_minimum_required(VERSION 3.5)
# project(can_transceiver)

# # Default to C++14
# if(NOT CMAKE_CXX_STANDARD)
# set(CMAKE_CXX_STANDARD 14)
# endif()

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)

# add_executable(talker src/can_transceiver.cpp)
# ament_target_dependencies(talker rclcpp std_msgs)

# install(TARGETS
# talker
# DESTINATION lib/${PROJECT_NAME})

# ament_package()

# # can_transceiver_ros_intf.cpp (subscriber to /Simulator)
# add_executable(listener src/can_transceiver_ros_intf.cpp)
# ament_target_dependencies(listener rclcpp std_msgs)

# install(TARGETS
# talker
# listener
# DESTINATION lib/${PROJECT_NAME})
