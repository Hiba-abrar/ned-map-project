cmake_minimum_required(VERSION 3.10)
project(NEDCampusNavigator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Threads REQUIRED)

# Add executable
add_executable(app_server main.cpp)

# Link libraries
target_link_libraries(app_server Threads::Threads)

# Platform-specific libraries
if(WIN32)
    target_link_libraries(app_server ws2_32 wsock32)
endif()

# Set output directory
set_target_properties(app_server PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}
)

# Copy static files to build directory
configure_file(${CMAKE_SOURCE_DIR}/index.html ${CMAKE_BINARY_DIR}/index.html COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/style.css ${CMAKE_BINARY_DIR}/style.css COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/script.js ${CMAKE_BINARY_DIR}/script.js COPYONLY)