set(module example)

# define external dependencies with link_libs and inc_dirs variables
set(link_libs
)

set(inc_dirs
)

# Create module library
set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/cached_fib.cpp
)
make_lib(${module} "${srcs}" "${link_libs}" "${inc_dirs}")

# Create module ROS executable
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/cached_fib_subscriber.cpp
)
make_ros_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_cached_fib.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}")
