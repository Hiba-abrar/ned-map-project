@echo off
echo Compiling NED Navigator...
g++ -o server.exe simple.cpp -lws2_32
if %errorlevel% equ 0 (
    echo Success! Starting server...
    server.exe
) else (
    echo Failed to compile
    pause
)