@echo off
echo 🎤 Building Voice-Driven NED Campus Navigator...
echo.

REM Create build directory
if not exist voice_build mkdir voice_build
cd voice_build

REM Copy required files
copy ..\voice_main.cpp .
copy ..\simple_server.h .
copy ..\json.hpp .
copy ..\voice_navigator.html .

REM Compile
echo 🔨 Compiling with enhanced voice features...
g++ -std=c++17 -O2 -o voice_navigator.exe voice_main.cpp -lws2_32 -pthread

if %errorlevel% equ 0 (
    echo.
    echo ✅ Build successful!
    echo.
    echo 🎤 Voice-Driven Features:
    echo    ✓ Voice input for start and destination
    echo    ✓ Step-based directions (left, right, forward)
    echo    ✓ Dynamic marker movement on map
    echo    ✓ Voice commands: "next", "repeat"
    echo    ✓ Meters to steps conversion (1m = 1.25 steps)
    echo.
    echo 🚀 Starting Voice Navigator Server...
    echo 🌐 Open http://localhost:8080 in your browser
    echo 🎯 Click "Start Voice Navigation" and follow voice prompts
    echo.
    voice_navigator.exe
) else (
    echo.
    echo ❌ Build failed!
    echo Make sure you have MinGW-w64 installed with g++ in PATH
    echo Required files: voice_main.cpp, simple_server.h, json.hpp, voice_navigator.html
    pause
)

cd ..