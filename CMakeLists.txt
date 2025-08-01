# =============================================================================
# Kalman Filter Implementation - Modern C++ Project
# =============================================================================
# A professional implementation of a Kalman filter for real-time state 
# estimation with comprehensive matrix operations and exception-safe design.
#
# Author: Ronny Milleo
# License: See LICENSE file in project root
# =============================================================================

cmake_minimum_required(VERSION 3.28)

# Project definition with version and metadata
project(PIDTuner
    VERSION 1.0.0
    DESCRIPTION "Modern C++ PID Tuner Implementation"
    LANGUAGES CXX
)

# =============================================================================
# Project Configuration
# =============================================================================

# Set default build type to Release if not specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# =============================================================================
# C++ Standard and Compiler Requirements
# =============================================================================

# Require C++17 for modern features used in the implementation
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # Disable compiler-specific extensions

# =============================================================================
# Build Configuration Options
# =============================================================================

# Project options
option(PIDTUNER_BUILD_TESTS "Build unit tests" ON)
option(PIDTUNER_BUILD_EXAMPLES "Build example programs" ON)
option(PIDTUNER_ENABLE_WARNINGS "Enable compiler warnings" ON)
option(PIDTUNER_WARNINGS_AS_ERRORS "Treat warnings as errors" OFF)

# =============================================================================
# Compiler Flags and Warnings
# =============================================================================

# Enable compile commands export for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set output directories for better organization
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${CMAKE_BUILD_TYPE}/lib")

# Compiler-specific warning flags
if(PIDTUNER_ENABLE_WARNINGS)
    if(MSVC)
        set(PIDTUNER_WARNING_FLAGS /W4)
        if(PIDTUNER_WARNINGS_AS_ERRORS)
            list(APPEND PIDTUNER_WARNING_FLAGS /WX)
        endif()
    else()
        set(PIDTUNER_WARNING_FLAGS 
            -Wall -Wextra -Wpedantic
            -Wcast-align -Wcast-qual -Wctor-dtor-privacy
            -Wdisabled-optimization -Wformat=2 -Winit-self
            -Wmissing-declarations -Wmissing-include-dirs
            -Wold-style-cast -Woverloaded-virtual -Wredundant-decls
            -Wshadow -Wsign-conversion -Wsign-promo
            -Wstrict-overflow=5 -Wswitch-default -Wundef
        )
        if(PIDTUNER_WARNINGS_AS_ERRORS)
            list(APPEND PIDTUNER_WARNING_FLAGS -Werror)
        endif()
    endif()
endif()

# =============================================================================
# Main Library Target
# =============================================================================

# Create a library target for the Kalman filter implementation
add_library(pidtuner_lib STATIC
    
)

# Set target properties
set_target_properties(pidtuner_lib PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
)

# Apply compiler warnings to library
if(PIDTUNER_ENABLE_WARNINGS)
    target_compile_options(pidtuner_lib PRIVATE ${PIDTUNER_WARNING_FLAGS})
endif()

# Configure include directories
target_include_directories(pidtuner_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
)

# =============================================================================
# Main Executable Target
# =============================================================================

if(PIDTUNER_BUILD_EXAMPLES)
    # Main demonstration executable
    add_executable(pidtuner_demo
        pidtuner_demo.cpp
    )

    # Link with the Kalman library
    target_link_libraries(pidtuner_demo
        PRIVATE
            pidtuner_lib
    )

    # Apply compiler warnings to executable
    if(PIDTUNER_ENABLE_WARNINGS)
        target_compile_options(pidtuner_demo PRIVATE ${PIDTUNER_WARNING_FLAGS})
    endif()

    # Set target properties
    set_target_properties(pidtuner_demo PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        OUTPUT_NAME "pidtuner_demo"
    )
    
    # Extended Kalman Filter demonstration executable
    add_executable(pidtuner_demo
        pidtuner_demo.cpp
    )

    # Link with the Kalman library
    target_link_libraries(pidtuner_demo
        PRIVATE
            pidtuner_lib
    )

    # Apply compiler warnings to executable
    if(PIDTUNER_ENABLE_WARNINGS)
        target_compile_options(pidtuner_demo PRIVATE ${PIDTUNER_WARNING_FLAGS})
    endif()

    # Set target properties
    set_target_properties(pidtuner_demo PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        CXX_EXTENSIONS OFF
        OUTPUT_NAME "pidtuner_demo"
    )
endif()

# =============================================================================
# Testing Configuration
# =============================================================================

if(PIDTUNER_BUILD_TESTS)
    enable_testing()
    message(STATUS "Building tests enabled")
    add_subdirectory(tests)
endif()

# =============================================================================
# Installation Configuration
# =============================================================================

# Install library and headers
install(TARGETS pidtuner_lib
    EXPORT PIDTunerTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Install header files
install(FILES
    
    DESTINATION include
)

# Install executable if built
if(KALMAN_BUILD_EXAMPLES)
    install(TARGETS pidtuner_demo
        DESTINATION bin
    )
endif()

# =============================================================================
# Project Summary
# =============================================================================

message(STATUS "")
message(STATUS "=== Kalman Filter Project Configuration ===")
message(STATUS "Project:              ${PROJECT_NAME}")
message(STATUS "Version:              ${PROJECT_VERSION}")
message(STATUS "Build type:           ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard:         ${CMAKE_CXX_STANDARD}")
message(STATUS "Compiler:             ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "Build tests:          ${KALMAN_BUILD_TESTS}")
message(STATUS "Build examples:       ${KALMAN_BUILD_EXAMPLES}")
message(STATUS "Enable warnings:      ${KALMAN_ENABLE_WARNINGS}")
message(STATUS "Warnings as errors:   ${KALMAN_WARNINGS_AS_ERRORS}")
message(STATUS "Install prefix:       ${CMAKE_INSTALL_PREFIX}")
message(STATUS "============================================")
message(STATUS "")