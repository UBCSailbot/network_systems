set(module example)

# Create module library
set(srcs
    ${CMAKE_CURRENT_LIST_DIR}/src/cached_fib.cpp
)
# Make the header accessible to other modules
add_library(${module} ${srcs})
target_include_directories(${module} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc)

# Create module ROS executable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
set(bin_module bin_${module})
set(bin_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/src/cached_fib_subscriber.cpp
)
add_executable(${bin_module} ${bin_srcs})
ament_target_dependencies(${bin_module} rclcpp std_msgs)
target_include_directories(${bin_module} PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc)
install(TARGETS
    ${bin_module}
    DESTINATION lib/${PROJECT_NAME}
)
# Rename the output binary to just be the module name
set_target_properties(${bin_module} PROPERTIES OUTPUT_NAME ${module})

# Create unit test
set(test_module test_${module})
set(test_srcs
    ${srcs}
    ${CMAKE_CURRENT_LIST_DIR}/test/test_cached_fib.cpp
)
add_executable(${test_module} ${test_srcs})
target_include_directories(${test_module} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/inc)
target_link_libraries(${test_module} gtest_main)
# Make the unit test runnable with CTest (invoked via test.sh)
add_test(NAME ${test_module} COMMAND ${test_module})
