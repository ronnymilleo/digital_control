# =============================================================================
# App setup
# =============================================================================

cmake_minimum_required(VERSION 3.28)

add_library(app_gui STATIC
    ${CMAKE_SOURCE_DIR}/application/app_gui.cpp
    ${CMAKE_SOURCE_DIR}/application/imgui_layer.cpp
    ${CMAKE_SOURCE_DIR}/application/pid_tuning_window.cpp
    ${CMAKE_SOURCE_DIR}/application/leadlag_tuning_window.cpp
)

target_include_directories(app_gui 
    PUBLIC
        ${CMAKE_SOURCE_DIR}/application
        ${CMAKE_SOURCE_DIR}/library/include
        ${CMAKE_SOURCE_DIR}/vendor/stb
)

target_link_libraries(app_gui 
    PUBLIC 
        imgui
        implot
        digital_control
        spdlog
)

# =============================================================================
# End of file
# =============================================================================
