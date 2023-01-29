# Create module library
function(make_lib module srcs)
    add_library(${module} ${srcs})
    target_include_directories(
        ${module} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${PROTOBUF_INCLUDE_PATH}
        ${CMN_HDRS_INC_PATH}
    )
endfunction()

# Create module ROS executable
function(make_ros_exe module srcs)
    set(bin_module bin_${module})
    add_executable(${bin_module} ${srcs})
    ament_target_dependencies(${bin_module} rclcpp std_msgs)
    target_include_directories(
        ${bin_module} PUBLIC 
        ${CMAKE_CURRENT_LIST_DIR}/inc 
        ${PROTOBUF_INCLUDE_PATH}
        ${CMN_HDRS_INC_PATH}
    )
    install(TARGETS ${bin_module} DESTINATION lib/${PROJECT_NAME})
    # Rename the output binary to just be the module name
    set_target_properties(${bin_module} PROPERTIES OUTPUT_NAME ${module})
endfunction()

# Create unit test
function(make_unit_test module srcs)
    set(test_module test_${module})
    add_executable(${test_module} ${srcs})
    target_include_directories(
        ${test_module} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${PROTOBUF_INCLUDE_PATH}
        ${CMN_HDRS_INC_PATH}
    )
    target_link_libraries(${test_module} ${GTEST_LINK_LIBS})
    # Make the unit test runnable with CTest (invoked via test.sh)
    add_test(NAME ${test_module} COMMAND ${test_module})
endfunction()
