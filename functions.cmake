# Create module library
function(make_lib module srcs link_libs inc_dirs)
    add_library(${module} ${srcs})
    target_link_libraries(${module} PUBLIC ${link_libs})
    target_include_directories(
        ${module} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMN_HDRS_INC_PATH}
        ${inc_dirs}
    )
endfunction()

# Create module ROS executable
function(make_ros_exe module srcs link_libs inc_dirs)
    set(bin_module bin_${module})
    add_executable(${bin_module} ${srcs})
    ament_target_dependencies(${bin_module} PUBLIC rclcpp std_msgs)
    target_link_libraries(${bin_module} PUBLIC ${link_libs})
    target_include_directories(
        ${bin_module} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMN_HDRS_INC_PATH}
        ${inc_dirs}
    )
    install(TARGETS ${bin_module} DESTINATION lib/${PROJECT_NAME})
    # Rename the output binary to just be the module name
    set_target_properties(${bin_module} PROPERTIES OUTPUT_NAME ${module})
endfunction()

# Create unit test
function(make_unit_test module srcs link_libs inc_dirs)
    set(test_module test_${module})
    add_executable(${test_module} ${srcs})
    target_include_directories(
        ${test_module} PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}/inc
        ${CMN_HDRS_INC_PATH}
        ${inc_dirs}
    )
    target_link_libraries(${test_module} PUBLIC ${GTEST_LINK_LIBS} ${link_libs})
    # Make the unit test runnable with CTest (invoked via test.sh)
    add_test(NAME ${test_module} COMMAND ${test_module})
endfunction()
