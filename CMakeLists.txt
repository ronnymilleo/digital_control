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
option(DC_BUILD_TESTS "Build unit tests" OFF)
option(DC_BUILD_EXAMPLES "Build example programs" ON)
option(DC_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(DC_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
)

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
)

# Example executable
if(DC_BUILD_EXAMPLES)
    add_executable(sim_pid_first_order
        examples/sim_pid_first_order.cpp
    )
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
endif()

# Install
install(TARGETS digital_control
    EXPORT DigitalControlTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES
    controller.h
    pid.h
    plant.h
    DESTINATION include
)

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
