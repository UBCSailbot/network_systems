set(module local_transceiver)

set(link_libs
    ${PROTOBUF_LINK_LIBS}
)

set(inc_dirs
    ${PROTOBUF_INCLUDE_PATH}
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver.cpp
)

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/local_transceiver_ros_intf.cpp
)
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}")
