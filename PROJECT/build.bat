@echo off
echo Building NED Campus Navigator...

REM Create build directory
if not exist build mkdir build
cd build

REM Configure with CMake
cmake ..

REM Build the project
cmake --build .

echo Build complete! Run 'app_server.exe' to start the server.
pause