# =============================================================================
# Digital Control
# =============================================================================
# Simple project to simulate a PID controller driving a first-order plant.
# =============================================================================

cmake_minimum_required(VERSION 3.28)

project(DigitalControl
    VERSION 0.1.0
    DESCRIPTION "Generic digital controller application"
    LANGUAGES CXX
)

# Default build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(DC_BUILD_TESTS "Build unit tests" ON)
option(DC_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# Formatting: run clang-format before builds if available
# -----------------------------------------------------------------------------
find_program(CLANG_FORMAT NAMES clang-format)
set(DC_FORMAT_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/controller.h
    ${CMAKE_CURRENT_SOURCE_DIR}/exceptions.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pid.h
    ${CMAKE_CURRENT_SOURCE_DIR}/pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/plant.h
    ${CMAKE_CURRENT_SOURCE_DIR}/plant.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tuner.h
    ${CMAKE_CURRENT_SOURCE_DIR}/tuner.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/sim_pid_first_order.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/sim_pid_second_order.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_pid.cpp
)
if(CLANG_FORMAT)
    add_custom_target(format ALL
        COMMAND ${CMAKE_COMMAND} -E echo "Running clang-format on sources"
        COMMAND ${CLANG_FORMAT} -i ${DC_FORMAT_FILES}
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        COMMENT "Formatting source code with clang-format"
        VERBATIM
    )
else()
    message(STATUS "clang-format not found; skipping automatic source formatting")
    add_custom_target(format ALL
        COMMAND ${CMAKE_COMMAND} -E echo "clang-format not found, skipping formatting"
    )
endif()

# Copy compile_commands.json to source dir for clangd after each build
add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Copying compile_commands.json to source dir for clangd"
)

# Output dirs
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${BUILD_TYPE_LOWER}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${BUILD_TYPE_LOWER}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${BUILD_TYPE_LOWER}/lib")

# Library
add_library(digital_control STATIC
    pid.cpp
    plant.cpp
    tuner.cpp
)
add_dependencies(digital_control format)

set_target_properties(digital_control PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

target_include_directories(digital_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/nlohmann_json/single_include
)

# Tests
if(DC_BUILD_TESTS)
    enable_testing()
    # Bring in vendored GoogleTest
    add_subdirectory(vendor/googletest)
    add_executable(digital_control_tests
        tests/test_pid.cpp
    )
    add_dependencies(digital_control_tests format)
    target_link_libraries(digital_control_tests PRIVATE gtest_main digital_control)
    add_test(NAME digital_control_tests COMMAND digital_control_tests)
endif()

# Summary
message(STATUS "")
message(STATUS "=== Digital Control Project Configuration ===")
message(STATUS "Project:              ${PROJECT_NAME}")
message(STATUS "Version:              ${PROJECT_VERSION}")
message(STATUS "Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:         ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler:             ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build tests:          ${DC_BUILD_TESTS}")
message(STATUS "============================================")
message(STATUS "")
