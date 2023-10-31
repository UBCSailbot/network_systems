set(lib virtual_iridium_server)

set(srcs
    virtual_iridium_server.h
)

set(link_libs
)

set(inc_dirs
    .
)

set(compile_defs
    RUN_VIRTUAL_IRIDIUM_SCRIPT_PATH="$ENV{ROS_WORKSPACE}/src/network_systems/scripts/run_virtual_iridium.sh"
)

make_lib(${lib} "${srcs}" "${link_libs}" "${inc_dirs}" "${compile_defs}")
