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

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# =============================================================================
# Default build type
# =============================================================================
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# Options
# =============================================================================

option(DC_BUILD_TESTS "Build unit tests" OFF)

# =============================================================================
# Formatting: run clang-format before builds if available
# =============================================================================
find_program(CLANG_FORMAT NAMES clang-format)
set(DC_FORMAT_FILES
    ${CMAKE_SOURCE_DIR}/main.cpp
    ${CMAKE_SOURCE_DIR}/application/app_gui.cpp
    ${CMAKE_SOURCE_DIR}/application/app_gui.h
    ${CMAKE_SOURCE_DIR}/application/imgui_layer.cpp
    ${CMAKE_SOURCE_DIR}/application/imgui_layer.h
    ${CMAKE_SOURCE_DIR}/application/leadlag_tuning_window.cpp
    ${CMAKE_SOURCE_DIR}/application/leadlag_tuning_window.h
    ${CMAKE_SOURCE_DIR}/core/controller.h
    ${CMAKE_SOURCE_DIR}/core/exceptions.h
    ${CMAKE_SOURCE_DIR}/core/pid.h
    ${CMAKE_SOURCE_DIR}/core/pid.cpp
    ${CMAKE_SOURCE_DIR}/core/leadlag.h
    ${CMAKE_SOURCE_DIR}/core/leadlag.cpp
    ${CMAKE_SOURCE_DIR}/core/plant.h
    ${CMAKE_SOURCE_DIR}/core/plant.cpp
    ${CMAKE_SOURCE_DIR}/core/tuner.h
    ${CMAKE_SOURCE_DIR}/core/tuner.cpp
    ${CMAKE_SOURCE_DIR}/examples/sim_pid_first_order.cpp
    ${CMAKE_SOURCE_DIR}/examples/sim_pid_second_order.cpp
    ${CMAKE_SOURCE_DIR}/examples/sim_leadlag_first_order.cpp
    ${CMAKE_SOURCE_DIR}/tests/test_pid.cpp
    ${CMAKE_SOURCE_DIR}/tests/test_leadlag.cpp
    ${CMAKE_SOURCE_DIR}/tests/test_plant.cpp
    ${CMAKE_SOURCE_DIR}/tests/test_integration.cpp
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

# =============================================================================
# Add subdirectories
# =============================================================================

add_subdirectory(vendor)
add_subdirectory(examples)
add_subdirectory(application)

# =============================================================================
# Copy compile_commands.json to source dir for clangd after each build
# =============================================================================
add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_BINARY_DIR}/compile_commands.json
            ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Copying compile_commands.json to source dir for clangd"
)

# =============================================================================
# Output dirs
# =============================================================================
string(TOLOWER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_LOWER)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${BUILD_TYPE_LOWER}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${BUILD_TYPE_LOWER}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/${BUILD_TYPE_LOWER}/lib")

# =============================================================================
# GUI Application
# =============================================================================

# Add Windows resource file for icon on Windows
if(WIN32)
    set(WINDOWS_RESOURCES ${CMAKE_SOURCE_DIR}/resources.rc)
else()
    set(WINDOWS_RESOURCES "")
endif()

add_executable(digital_control
    ${CMAKE_SOURCE_DIR}/main.cpp
    ${WINDOWS_RESOURCES}
)

target_include_directories(digital_control
    PRIVATE
        ${CMAKE_SOURCE_DIR}/application
        ${CMAKE_SOURCE_DIR}/core
)

target_link_libraries(digital_control 
    PRIVATE 
        app_gui
)

# Copy assets folder to output directory after build
add_custom_command(TARGET digital_control POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/assets
    $<TARGET_FILE_DIR:digital_control>/assets
    COMMENT "Copying assets to output directory"
)

# =============================================================================
# Library
# =============================================================================

add_library(digital_control_lib STATIC
    ${CMAKE_SOURCE_DIR}/core/pid.cpp
    ${CMAKE_SOURCE_DIR}/core/leadlag.cpp
    ${CMAKE_SOURCE_DIR}/core/plant.cpp
    ${CMAKE_SOURCE_DIR}/core/tuner.cpp
)

add_dependencies(digital_control_lib format)

target_include_directories(digital_control_lib
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/vendor/nlohmann_json/include
)

target_link_libraries(digital_control_lib
    PUBLIC
        nlohmann_json::nlohmann_json
)

# =============================================================================
# Tests
# =============================================================================

if(DC_BUILD_TESTS)
    enable_testing()
    # Bring in vendored GoogleTest
    add_subdirectory(vendor/googletest)
    add_executable(digital_control_tests
        tests/test_pid.cpp
        tests/test_leadlag.cpp
        tests/test_plant.cpp
        tests/test_integration.cpp
    )
    add_dependencies(digital_control_tests format)
    target_include_directories(digital_control_tests PRIVATE ${CMAKE_SOURCE_DIR}/core)
    target_link_libraries(digital_control_tests PRIVATE gtest_main digital_control_lib)
    add_test(NAME digital_control_tests COMMAND digital_control_tests)
endif()

# =============================================================================
# Summary
# =============================================================================
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

# =============================================================================
# End of file
# =============================================================================
