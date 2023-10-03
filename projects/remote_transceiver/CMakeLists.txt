set (module remote_transceiver)

set(link_libs
    ${PROTOBUF_LINK_LIBS}
    mongo::mongocxx_shared
    mongo::bsoncxx_shared
)

set(inc_dirs
    ${PROTOBUF_INCLUDE_PATH}
    ${LIBMONGOCXX_INCLUDE_DIRS}
    ${LIBBSONCXX_INCLUDE_DIRS}
)

set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/sailbot_db.cpp
)

set(compile_defs
    MONGODB_CONN_STR="$ENV{MONGODB_CONN_STR}"
)

set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/remote_transceiver.cpp
)

# Make executable (not ROS)
make_exe(${module} "${bin_srcs}" "${link_libs}" "${inc_dirs}")

# Create unit test
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_remote_transceiver.cpp
)
make_unit_test(${module} "${test_srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
