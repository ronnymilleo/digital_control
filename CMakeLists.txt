# =============================================================================
# Digital Control - Minimal PID Simulation Project
# =============================================================================
# Simple project to simulate a PID controller driving a first-order plant.
# =============================================================================

cmake_minimum_required(VERSION 3.28)

project(DigitalControl
    VERSION 0.1.0
    DESCRIPTION "Generic digital controller library with PID and simple plant simulation"
    LANGUAGES CXX
)

# Default build type
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(DC_BUILD_TESTS "Build unit tests" ON)
option(DC_BUILD_EXAMPLES "Build example programs" ON)
option(DC_ENABLE_WARNINGS "Enable compiler warnings" ON)
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
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/lib")

# Warnings
if(DC_ENABLE_WARNINGS)
    if(MSVC)
        set(DC_WARNING_FLAGS /W4)
        if(DC_WARNINGS_AS_ERRORS)
            list(APPEND DC_WARNING_FLAGS /WX)
        endif()
    else()
        set(DC_WARNING_FLAGS 
            -Wall -Wextra -Wpedantic
            -Wcast-align -Wcast-qual
            -Wdisabled-optimization -Wformat=2 -Winit-self
            -Wmissing-declarations -Wmissing-include-dirs
            -Wold-style-cast -Woverloaded-virtual -Wredundant-decls
            -Wshadow -Wsign-conversion -Wsign-promo
            -Wstrict-overflow=5 -Wswitch-default -Wundef
        )
        if(DC_WARNINGS_AS_ERRORS)
            list(APPEND DC_WARNING_FLAGS -Werror)
        endif()
    endif()
endif()

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

if(DC_ENABLE_WARNINGS)
    target_compile_options(digital_control PRIVATE ${DC_WARNING_FLAGS})
endif()

target_include_directories(digital_control
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/nlohmann_json/single_include
)

# Example executables
if(DC_BUILD_EXAMPLES)
    add_executable(sim_pid_first_order
        examples/sim_pid_first_order.cpp
    )
    add_dependencies(sim_pid_first_order format)
    target_link_libraries(sim_pid_first_order PRIVATE digital_control)
    if(DC_ENABLE_WARNINGS)
        target_compile_options(sim_pid_first_order PRIVATE ${DC_WARNING_FLAGS})
    endif()
    set_target_properties(sim_pid_first_order PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        OUTPUT_NAME "sim_pid_first_order"
    )

    add_executable(sim_pid_second_order
        examples/sim_pid_second_order.cpp
    )
    add_dependencies(sim_pid_second_order format)
    target_link_libraries(sim_pid_second_order PRIVATE digital_control)
    if(DC_ENABLE_WARNINGS)
        target_compile_options(sim_pid_second_order PRIVATE ${DC_WARNING_FLAGS})
    endif()
    set_target_properties(sim_pid_second_order PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        OUTPUT_NAME "sim_pid_second_order"
    )
endif()

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
    if(DC_ENABLE_WARNINGS)
        target_compile_options(digital_control_tests PRIVATE ${DC_WARNING_FLAGS})
    endif()
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
message(STATUS "Build examples:       ${DC_BUILD_EXAMPLES}")
message(STATUS "Enable warnings:      ${DC_ENABLE_WARNINGS}")
message(STATUS "Warnings as errors:   ${DC_WARNINGS_AS_ERRORS}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "============================================")
message(STATUS "")
