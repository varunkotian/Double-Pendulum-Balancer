cmake_minimum_required(VERSION 3.16)
project(MPC_DoublePendulum)

set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set output directories to keep build folder organized
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Source files
set(SOURCES
    src/main.cpp
    src/DoublePendulum.cpp
    src/MPC_Controller.cpp
    src/ImGuiRenderer.cpp
    external/imgui.cpp
    external/imgui_draw.cpp
    external/imgui_demo.cpp
    external/imgui_widgets.cpp
    external/imgui_tables.cpp
    external/imgui_impl_dx11.cpp
    external/imgui_impl_win32.cpp
)

# Header files
set(HEADERS
    include/DoublePendulum.h
    include/MPC_Controller.h
    include/ImGuiRenderer.h
)

# Create executable
add_executable(MPC_DoublePendulum ${SOURCES} ${HEADERS})

# Include directories
target_include_directories(MPC_DoublePendulum PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/external
)

# Link libraries
target_link_libraries(MPC_DoublePendulum PRIVATE
    d3d11
    d3dcompiler
    dxgi
    dwmapi
)
