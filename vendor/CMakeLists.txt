# =============================================================================
# Vendor libraries setup
# =============================================================================

cmake_minimum_required(VERSION 3.28)

find_package(OpenGL REQUIRED)

# Force GLFW to build as static library
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(GLFW_LIBRARY_TYPE "STATIC" CACHE STRING "" FORCE)

add_subdirectory(glfw)
add_subdirectory(nlohmann_json)
add_subdirectory(spdlog)

add_library(imgui STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_demo.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_draw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_tables.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/imgui_widgets.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends/imgui_impl_opengl3.cpp
)

target_include_directories(imgui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/imgui/backends
    ${CMAKE_CURRENT_SOURCE_DIR}/glfw/include
)

target_link_libraries(imgui 
    PUBLIC 
        glfw 
        OpenGL::GL
)

# =============================================================================
# End of file
# =============================================================================
