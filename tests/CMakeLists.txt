# =============================================================================
# Tests setup
# =============================================================================

cmake_minimum_required(VERSION 3.28)

enable_testing()

add_executable(digital_control_tests
    test_pid.cpp
    test_integration.cpp
    test_leadlag.cpp
    test_plant.cpp
)

add_dependencies(digital_control_tests format)

target_include_directories(digital_control_tests 
    PRIVATE
        ${CMAKE_SOURCE_DIR}/library/include
)

target_link_libraries(digital_control_tests 
    PRIVATE 
        gtest_main 
        digital_control
)

add_test(NAME digital_control_tests COMMAND digital_control_tests)

# =============================================================================
# End of file
# =============================================================================
