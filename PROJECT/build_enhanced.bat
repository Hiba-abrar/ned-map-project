@echo off
echo Building Enhanced NED Campus Navigator...

REM Create build directory
if not exist enhanced_build mkdir enhanced_build
cd enhanced_build

REM Copy required files
copy ..\enhanced_main.cpp .
copy ..\simple_server.h .
copy ..\json.hpp .
copy ..\enhanced_navigator.html .

REM Compile directly
echo Compiling with g++...
g++ -std=c++17 -O2 -o enhanced_navigator.exe enhanced_main.cpp -lws2_32 -pthread

if %errorlevel% equ 0 (
    echo.
    echo ✅ Build successful!
    echo.
    echo 🚀 Starting Enhanced NED Campus Navigator...
    echo 📱 Features: GPS Integration, Voice Commands, Realistic Routing
    echo 🌐 Open http://localhost:8080 in your browser
    echo.
    enhanced_navigator.exe
) else (
    echo.
    echo ❌ Build failed!
    echo Make sure you have MinGW-w64 installed with g++ in PATH
    pause
)

cd ..