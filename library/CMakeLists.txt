# =============================================================================
# Library
# =============================================================================

add_library(digital_control STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/pid.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/leadlag.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/plant.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tuner.cpp
)

add_dependencies(digital_control format)

target_include_directories(digital_control
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/vendor/nlohmann_json/include
)

target_link_libraries(digital_control
    PUBLIC
        nlohmann_json::nlohmann_json
)
