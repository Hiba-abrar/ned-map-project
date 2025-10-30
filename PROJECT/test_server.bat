@echo off
echo Testing NED Campus Navigator Server...
echo.

echo Starting server in background...
start /B build\app_server.exe

echo Waiting for server to start...
timeout /t 3 /nobreak > nul

echo.
echo Testing API endpoints:
echo.

echo 1. Testing GET /api/locations
curl -s http://localhost:8080/api/locations | head -c 200
echo.
echo.

echo 2. Testing POST /api/route
curl -s -X POST http://localhost:8080/api/route -H "Content-Type: application/json" -d "{\"start\":\"NED University Main Gate\",\"end\":\"NED University Library\"}" | head -c 300
echo.
echo.

echo 3. Testing static file serving (GET /)
curl -s http://localhost:8080/ | head -c 100
echo.
echo.

echo Test complete! 
echo Open http://localhost:8080 in your browser to use the interface.
echo Press any key to stop the server...
pause > nul

echo Stopping server...
taskkill /F /IM app_server.exe > nul 2>&1