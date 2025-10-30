@echo off
echo Compiling simple version...
g++ -std=c++17 -o navigator.exe main.cpp -lws2_32
if %errorlevel% equ 0 (
    echo Compiled successfully!
    echo Starting server...
    start navigator.exe
    echo Open http://localhost:8080 in your browser
) else (
    echo Compilation failed!
)
pause