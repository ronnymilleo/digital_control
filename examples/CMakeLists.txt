# =============================================================================
# Digital Control Examples
# =============================================================================
# Example applications demonstrating PID controller simulations
# =============================================================================

cmake_minimum_required(VERSION 3.28)

# =============================================================================
# Example: PID First Order System Simulation
# =============================================================================
add_executable(sim_pid_first_order
    sim_pid_first_order.cpp
)

add_dependencies(sim_pid_first_order format)

target_include_directories(sim_pid_first_order
    PRIVATE
        ${CMAKE_SOURCE_DIR}/core
)

target_link_libraries(sim_pid_first_order
    PRIVATE
        digital_control_lib
)

set_target_properties(sim_pid_first_order PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# =============================================================================
# Example: PID Second Order System Simulation
# =============================================================================
add_executable(sim_pid_second_order
    sim_pid_second_order.cpp
)

add_dependencies(sim_pid_second_order format)

target_include_directories(sim_pid_second_order
    PRIVATE
        ${CMAKE_SOURCE_DIR}/core
)

target_link_libraries(sim_pid_second_order
    PRIVATE
        digital_control_lib
)

set_target_properties(sim_pid_second_order PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# =============================================================================
# Example: Lead/Lag Compensator Simulation
# =============================================================================
add_executable(sim_leadlag_first_order
    sim_leadlag_first_order.cpp
)

add_dependencies(sim_leadlag_first_order format)

target_include_directories(sim_leadlag_first_order
    PRIVATE
        ${CMAKE_SOURCE_DIR}/core
)

target_link_libraries(sim_leadlag_first_order
    PRIVATE
        digital_control_lib
)

set_target_properties(sim_leadlag_first_order PROPERTIES
    CXX_STANDARD 23
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
)

# =============================================================================
# Optional: GUI examples if ImGui is available
# =============================================================================
if(TARGET imgui)
    # Add GUI-based examples here if needed in the future
    # Example:
    # add_executable(sim_pid_gui
    #     sim_pid_gui.cpp
    # )
    # target_link_libraries(sim_pid_gui
    #     PRIVATE
    #         digital_control
    #         imgui
    # )
endif()

# =============================================================================
# End of file
# =============================================================================
