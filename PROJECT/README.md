# NED Campus Navigator

A web-based navigation system for NED University campus with voice commands and interactive mapping.

## Features

- Interactive map with campus locations
- Route finding using Dijkstra's algorithm
- Voice command support
- Turn-by-turn navigation
- Real-time route visualization
- Text-to-speech guidance

## Build Requirements

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2017+)
- CMake 3.10+
- Modern web browser with JavaScript support

## Build Instructions

### Linux/macOS

```bash
chmod +x build.sh
./build.sh
cd build
./app_server
```

### Windows

```cmd
build.bat
cd build
app_server.exe
```

### Manual Build

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Usage

1. Start the server:
   ```bash
   ./app_server  # Linux/macOS
   app_server.exe  # Windows
   ```

2. Open your browser and navigate to: `http://localhost:8080`

3. Use the interface to:
   - Enter start and destination locations
   - Click "Find Route" to calculate path
   - Use voice commands like "Navigate from Library to Main Gate"
   - Follow turn-by-turn directions

## API Endpoints

### GET /api/locations
Returns all available campus locations.

**Response:**
```json
[
  {
    "id": 0,
    "name": "NED University Admin Block",
    "lat": 24.9328525,
    "lon": 67.1099341
  }
]
```

### POST /api/route
Calculates route between two locations.

**Request:**
```json
{
  "start": "NED University Main Gate",
  "end": "NED University Library"
}
```

**Response:**
```json
{
  "success": true,
  "path": [
    {
      "name": "NED University Main Gate",
      "lat": 24.9299167,
      "lon": 67.1156389
    }
  ],
  "steps": [
    {
      "instruction": "Continue to NED University Library",
      "distance": 250
    }
  ],
  "total_distance": 250
}
```

## Test Examples

### Using curl

1. Get locations:
```bash
curl http://localhost:8080/api/locations
```

2. Find route:
```bash
curl -X POST http://localhost:8080/api/route \
  -H "Content-Type: application/json" \
  -d '{"start":"NED University Main Gate","end":"NED University Library"}'
```

### Using Browser

1. Open `http://localhost:8080`
2. Enter "NED University Main Gate" as start location
3. Enter "NED University Library" as destination
4. Click "Find Route"
5. Expected: Route displayed on map with turn-by-turn directions

## Voice Commands

- "Navigate from [location] to [location]"
- "Go to [location]"
- "Find route to [location]"

## File Structure

```
PROJECT/
├── main.cpp           # C++ server with navigation logic
├── index.html         # Frontend interface
├── script.js          # JavaScript client code
├── style.css          # Styling
├── httplib.h          # HTTP server library
├── json.hpp           # JSON library (download required)
├── CMakeLists.txt     # Build configuration
├── build.sh           # Unix build script
├── build.bat          # Windows build script
└── README.md          # This file
```

## Dependencies

- **httplib.h**: Single-header HTTP server library (included)
- **nlohmann/json**: JSON library (download required)

To download json.hpp:
```bash
curl -o json.hpp https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp
```

## Troubleshooting

1. **Port 8080 already in use**: Change port in main.cpp
2. **JSON library missing**: Download json.hpp as shown above
3. **Build fails**: Ensure C++17 support and CMake 3.10+
4. **Voice not working**: Use HTTPS or localhost only
5. **Map not loading**: Check internet connection for map tiles

## Architecture

- **Backend**: C++ HTTP server using httplib
- **Frontend**: HTML5 with Leaflet.js for mapping
- **Communication**: REST API with JSON
- **Navigation**: Dijkstra's algorithm for pathfinding
- **Voice**: Web Speech API for recognition and synthesis