set(module can_transceiver)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/can_frame_parser.cpp
)

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/can_transceiver_ros_intf.cpp
)
make_ros_exe(${module} "${bin_srcs}")
