set(module ais_transceiver)

# define external dependencies with link_libs and inc_dirs variables
set(link_libs
)

set(inc_dirs
)

set(compile_defs
)

# Create module library
set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/mock_ais_sim.cpp
)
make_lib(${module} "${srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/ais_transceiver_ros_intf.cpp
)
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_ais_transceiver.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
