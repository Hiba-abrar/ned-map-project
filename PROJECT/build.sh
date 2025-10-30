#!/bin/bash

echo "Building NED Campus Navigator..."

# Create build directory
mkdir -p build
cd build

# Configure with CMake
cmake ..

# Build the project
cmake --build .

echo "Build complete! Run './app_server' to start the server."