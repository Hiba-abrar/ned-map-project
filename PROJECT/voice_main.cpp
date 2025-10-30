#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <unordered_map>
#include <map>
#include <cmath>
#include <thread>
#include <chrono>
#include "json.hpp"
#include "simple_server.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using json = nlohmann::json;

// Enhanced location database
struct Location {
    string name;
    double lat, lon;
    vector<string> keywords;
};

vector<Location> campus_locations = {
    {"NED University Main Gate", 24.9299167, 67.1156389, {"main", "gate", "entrance"}},
    {"NED University Library", 24.9335294, 67.1110976, {"library", "books", "study"}},
    {"NED University Admin Block", 24.9328525, 67.1099341, {"admin", "office", "administration"}},
    {"CSIT Labs", 24.9313470, 67.1139814, {"csit", "computer", "lab", "cs"}},
    {"Main Auditorium", 24.9320192, 67.1125931, {"auditorium", "hall", "events"}},
    {"DMS Cafeteria", 24.9326224, 67.1144837, {"cafeteria", "food", "dining", "dms"}},
    {"Civil Engineering Class Rooms", 24.9313328, 67.1126852, {"civil", "engineering", "classroom"}},
    {"Survey Lab", 24.9318691, 67.1135877, {"survey", "lab", "surveying"}},
    {"Mosque", 24.9344365, 67.1109842, {"mosque", "prayer", "masjid"}},
    {"Basketball Court", 24.9325, 67.1164, {"basketball", "court", "sports"}},
    {"Tennis Court", 24.9326117, 67.1163778, {"tennis", "court"}},
    {"Football Ground", 24.9315048, 67.1166501, {"football", "ground", "field"}},
    {"Mathematics Department", 24.9309500, 67.1139000, {"math", "mathematics", "dept"}},
    {"Mechanical Engineering Department", 24.9317800, 67.1120400, {"mechanical", "engineering", "mech"}},
    {"Environmental Engineering", 24.9345252, 67.1125900, {"environmental", "env"}},
    {"Electrical Engineering Department", 24.9326500, 67.1124000, {"electrical", "ee", "elect"}}
};

// Navigation instruction structure
struct NavigationStep {
    string instruction;
    int steps;
    string direction;
    string action;
    string destination;
    double lat, lon;
    string icon;
};

class VoiceNavigationSystem {
private:
    vector<vector<double>> distances;
    vector<vector<int>> connections;
    
public:
    VoiceNavigationSystem() {
        buildGraph();
    }
    
    void buildGraph() {
        int n = campus_locations.size();
        distances.resize(n, vector<double>(n, 0));
        connections.resize(n);
        
        // Calculate distances and build connections
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    double dist = haversine(
                        campus_locations[i].lat, campus_locations[i].lon,
                        campus_locations[j].lat, campus_locations[j].lon
                    );
                    distances[i][j] = dist;
                    
                    // Connect nearby locations (within 400m)
                    if (dist < 400) {
                        connections[i].push_back(j);
                    }
                }
            }
        }
    }
    
    double haversine(double lat1, double lon1, double lat2, double lon2) {
        const double R = 6371000; // Earth radius in meters
        double dLat = (lat2 - lat1) * M_PI / 180.0;
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        lat1 *= M_PI / 180.0;
        lat2 *= M_PI / 180.0;
        double a = sin(dLat / 2) * sin(dLat / 2) +
                   cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        return R * c;
    }
    
    vector<int> findShortestPath(int start, int end) {
        int n = campus_locations.size();
        vector<double> dist(n, numeric_limits<double>::infinity());
        vector<int> parent(n, -1);
        vector<bool> visited(n, false);
        
        dist[start] = 0;
        
        for (int count = 0; count < n - 1; count++) {
            int u = -1;
            for (int v = 0; v < n; v++) {
                if (!visited[v] && (u == -1 || dist[v] < dist[u])) {
                    u = v;
                }
            }
            
            if (dist[u] == numeric_limits<double>::infinity()) break;
            visited[u] = true;
            
            for (int v : connections[u]) {
                if (!visited[v] && dist[u] + distances[u][v] < dist[v]) {
                    dist[v] = dist[u] + distances[u][v];
                    parent[v] = u;
                }
            }
        }
        
        // Reconstruct path
        vector<int> path;
        if (dist[end] == numeric_limits<double>::infinity()) {
            return path; // No path found
        }
        
        int current = end;
        while (current != -1) {
            path.push_back(current);
            current = parent[current];
        }
        reverse(path.begin(), path.end());
        return path;
    }
    
    double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
        double dLon = (lon2 - lon1) * M_PI / 180.0;
        lat1 *= M_PI / 180.0;
        lat2 *= M_PI / 180.0;
        
        double y = sin(dLon) * cos(lat2);
        double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
        
        double bearing = atan2(y, x) * 180.0 / M_PI;
        return fmod(bearing + 360.0, 360.0);
    }
    
    string bearingToDirection(double bearing) {
        const vector<string> directions = {"north", "northeast", "east", "southeast", 
                                         "south", "southwest", "west", "northwest"};
        int index = (int)round(bearing / 45.0) % 8;
        return directions[index];
    }
    
    string calculateTurnDirection(double prevBearing, double currentBearing) {
        double angleDiff = currentBearing - prevBearing;
        if (angleDiff > 180) angleDiff -= 360;
        if (angleDiff < -180) angleDiff += 360;
        
        if (angleDiff > 45 && angleDiff < 135) return "Turn right and move";
        else if (angleDiff > -135 && angleDiff < -45) return "Turn left and move";
        else if (abs(angleDiff) > 135) return "Turn around and move";
        else return "Continue straight";
    }
    
    string getDirectionIcon(const string& action) {
        if (action.find("right") != string::npos) return "↗️";
        else if (action.find("left") != string::npos) return "↖️";
        else if (action.find("around") != string::npos) return "🔄";
        else if (action.find("straight") != string::npos) return "⬆️";
        else return "➡️";
    }
    
    vector<NavigationStep> generateStepInstructions(const vector<int>& path) {
        vector<NavigationStep> instructions;
        
        if (path.size() < 2) return instructions;
        
        double prevBearing = 0;
        
        for (int i = 0; i < path.size() - 1; i++) {
            NavigationStep step;
            
            int currentIdx = path[i];
            int nextIdx = path[i + 1];
            
            double distance = distances[currentIdx][nextIdx];
            step.steps = (int)round(distance * 1.25); // Convert meters to steps
            
            double bearing = calculateBearing(
                campus_locations[currentIdx].lat, campus_locations[currentIdx].lon,
                campus_locations[nextIdx].lat, campus_locations[nextIdx].lon
            );
            
            step.direction = bearingToDirection(bearing);
            step.destination = campus_locations[nextIdx].name;
            step.lat = campus_locations[nextIdx].lat;
            step.lon = campus_locations[nextIdx].lon;
            
            if (i == 0) {
                step.action = "Move";
                step.icon = "🚀";
            } else {
                step.action = calculateTurnDirection(prevBearing, bearing);
                step.icon = getDirectionIcon(step.action);
            }
            
            step.instruction = step.action + " " + to_string(step.steps) + 
                             " steps " + step.direction + " toward " + step.destination;
            
            instructions.push_back(step);
            prevBearing = bearing;
        }
        
        return instructions;
    }
    
    int findLocationByVoice(const string& voiceInput) {
        string input = voiceInput;
        transform(input.begin(), input.end(), input.begin(), ::tolower);
        
        // Direct name match
        for (int i = 0; i < campus_locations.size(); i++) {
            string name = campus_locations[i].name;
            transform(name.begin(), name.end(), name.begin(), ::tolower);
            if (name == input) return i;
        }
        
        // Keyword matching
        for (int i = 0; i < campus_locations.size(); i++) {
            for (const string& keyword : campus_locations[i].keywords) {
                if (input.find(keyword) != string::npos) {
                    return i;
                }
            }
        }
        
        // Partial name matching
        for (int i = 0; i < campus_locations.size(); i++) {
            string name = campus_locations[i].name;
            transform(name.begin(), name.end(), name.begin(), ::tolower);
            
            vector<string> inputWords;
            stringstream ss(input);
            string word;
            while (ss >> word) {
                inputWords.push_back(word);
            }
            
            int matches = 0;
            for (const string& inputWord : inputWords) {
                if (name.find(inputWord) != string::npos && inputWord.length() > 2) {
                    matches++;
                }
            }
            
            if (matches >= 2 || (matches >= 1 && inputWords.size() <= 2)) {
                return i;
            }
        }
        
        return -1; // Not found
    }
    
    json getLocationsJson() {
        json locations = json::array();
        for (int i = 0; i < campus_locations.size(); i++) {
            locations.push_back({
                {"id", i},
                {"name", campus_locations[i].name},
                {"lat", campus_locations[i].lat},
                {"lon", campus_locations[i].lon},
                {"keywords", campus_locations[i].keywords}
            });
        }
        return locations;
    }
    
    json calculateVoiceRoute(const string& startVoice, const string& endVoice) {
        json result;
        
        int startId = findLocationByVoice(startVoice);
        int endId = findLocationByVoice(endVoice);
        
        if (startId == -1) {
            result["success"] = false;
            result["error"] = "Starting location not recognized";
            result["suggestion"] = "Try saying the full name or key words";
            return result;
        }
        
        if (endId == -1) {
            result["success"] = false;
            result["error"] = "Destination not recognized";
            result["suggestion"] = "Try saying the full name or key words";
            return result;
        }
        
        vector<int> path = findShortestPath(startId, endId);
        
        if (path.empty()) {
            result["success"] = false;
            result["error"] = "No route found between locations";
            return result;
        }
        
        vector<NavigationStep> steps = generateStepInstructions(path);
        
        // Build JSON response
        json pathJson = json::array();
        json stepsJson = json::array();
        int totalSteps = 0;
        
        for (int nodeId : path) {
            pathJson.push_back({
                {"name", campus_locations[nodeId].name},
                {"lat", campus_locations[nodeId].lat},
                {"lon", campus_locations[nodeId].lon}
            });
        }
        
        for (const NavigationStep& step : steps) {
            stepsJson.push_back({
                {"instruction", step.instruction},
                {"steps", step.steps},
                {"direction", step.direction},
                {"action", step.action},
                {"destination", step.destination},
                {"lat", step.lat},
                {"lon", step.lon},
                {"icon", step.icon}
            });
            totalSteps += step.steps;
        }
        
        result["success"] = true;
        result["path"] = pathJson;
        result["instructions"] = stepsJson;
        result["total_steps"] = totalSteps;
        result["total_distance"] = (int)(totalSteps * 0.8); // Convert back to meters
        result["start_location"] = campus_locations[startId].name;
        result["end_location"] = campus_locations[endId].name;
        result["instruction_count"] = steps.size();
        
        return result;
    }
};

// Global navigation system
VoiceNavigationSystem voiceNav;

// File reading utility
string readFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) return "";
    return string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
}

int main() {
    SimpleServer server;
    
    cout << "🎤 Voice-Driven NED Campus Navigator Server Starting..." << endl;
    cout << "Features:" << endl;
    cout << "- Voice input for start and destination" << endl;
    cout << "- Step-based directions with left/right/forward" << endl;
    cout << "- Dynamic marker movement" << endl;
    cout << "- Voice commands: next, repeat" << endl;
    cout << endl;
    
    // Serve main page
    server.Get("/", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        string content = readFile("voice_navigator.html");
        if (!content.empty()) {
            res.set_content(content, "text/html");
        } else {
            res.status = 404;
            res.body = "voice_navigator.html not found";
        }
    });
    
    // API: Get all locations with keywords
    server.Get("/api/locations", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        json locations = voiceNav.getLocationsJson();
        res.set_content(locations.dump(), "application/json");
    });
    
    // API: Find location by voice input
    server.Post("/api/find-location", [](const SimpleServer::Request& req, SimpleServer::Response& res) {
        try {
            json requestData = json::parse(req.body);
            string voiceInput = requestData["voice_input"];
            
            int locationId = voiceNav.findLocationByVoice(voiceInput);
            
            json result;
            if (locationId != -1) {
                result["success"] = true;
                result["location"] = {
                    {"id", locationId},
                    {"name", campus_locations[locationId].name},
                    {"lat", campus_locations[locationId].lat},
                    {"lon", campus_locations[locationId].lon}
                };
            } else {
                result["success"] = false;
                result["error"] = "Location not recognized";
                
                // Suggest similar locations
                json suggestions = json::array();
                for (const auto& loc : campus_locations) {
                    suggestions.push_back(loc.name);
                }
                result["suggestions"] = suggestions;
            }
            
            res.set_content(result.dump(), "application/json");
            
        } catch (const exception& e) {
            json error = {{"success", false}, {"error", "Invalid request format"}};
            res.set_content(error.dump(), "application/json");
        }
    });
    
    // API: Calculate voice-driven route
    server.Post("/api/voice-route", [](const SimpleServer::Request& req, SimpleServer::Response& res) {
        try {
            json requestData = json::parse(req.body);
            string startVoice = requestData["start_voice"];
            string endVoice = requestData["end_voice"];
            
            json result = voiceNav.calculateVoiceRoute(startVoice, endVoice);
            res.set_content(result.dump(), "application/json");
            
        } catch (const exception& e) {
            json error = {
                {"success", false},
                {"error", "Invalid request format"},
                {"details", e.what()}
            };
            res.set_content(error.dump(), "application/json");
        }
    });
    
    // Handle CORS
    server.Options(".*", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        // CORS headers already set in simple_server.h
    });
    
    cout << "Server running on http://localhost:8080" << endl;
    cout << "🎯 Open the URL in your browser to start voice navigation!" << endl;
    
    server.listen("localhost", 8080);
    
    return 0;
}