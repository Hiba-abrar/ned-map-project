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

// Enhanced location database with more details
struct Location {
    string name;
    double lat, lon;
    string type;
    string description;
    vector<string> aliases;
};

vector<Location> campus_locations = {
    {"NED University Main Gate", 24.9299167, 67.1156389, "entrance", "Main entrance to the university", {"main gate", "entrance", "gate"}},
    {"NED University Library", 24.9335294, 67.1110976, "academic", "Central library with study areas", {"library", "books", "study"}},
    {"NED University Admin Block", 24.9328525, 67.1099341, "administrative", "Administrative offices", {"admin", "office", "administration"}},
    {"CSIT Labs", 24.9313470, 67.1139814, "academic", "Computer Science labs", {"csit", "computer", "lab", "cs"}},
    {"Main Auditorium", 24.9320192, 67.1125931, "facility", "Main auditorium for events", {"auditorium", "hall", "events"}},
    {"DMS Cafeteria", 24.9326224, 67.1144837, "facility", "Dining and food services", {"cafeteria", "food", "dining", "dms"}},
    {"Civil Engineering Class Rooms", 24.9313328, 67.1126852, "academic", "Civil engineering department", {"civil", "engineering", "classroom"}},
    {"Survey Lab", 24.9318691, 67.1135877, "academic", "Surveying laboratory", {"survey", "lab", "surveying"}},
    {"Mosque", 24.9344365, 67.1109842, "religious", "Campus mosque", {"mosque", "prayer", "masjid"}},
    {"Basketball Court", 24.9325, 67.1164, "sports", "Basketball playing area", {"basketball", "court", "sports"}},
    {"Tennis Court", 24.9326117, 67.1163778, "sports", "Tennis playing area", {"tennis", "court"}},
    {"Football Ground", 24.9315048, 67.1166501, "sports", "Football field", {"football", "ground", "field"}},
    {"Mathematics Department", 24.9309500, 67.1139000, "academic", "Mathematics department", {"math", "mathematics", "dept"}},
    {"Mechanical Engineering Department", 24.9317800, 67.1120400, "academic", "Mechanical engineering", {"mechanical", "engineering", "mech"}},
    {"Environmental Engineering", 24.9345252, 67.1125900, "academic", "Environmental engineering", {"environmental", "env"}},
    {"Electrical Engineering Department", 24.9326500, 67.1124000, "academic", "Electrical engineering", {"electrical", "ee", "elect"}}
};

// Graph structure for pathfinding
struct Node {
    int id;
    string name;
    double lat, lon;
    vector<int> connections;
};

struct RouteStep {
    string instruction;
    double distance;
    string direction;
    double lat, lon;
};

class NavigationSystem {
private:
    vector<Node> nodes;
    vector<vector<double>> distances;
    
public:
    NavigationSystem() {
        buildGraph();
    }
    
    void buildGraph() {
        // Create nodes from locations
        for (int i = 0; i < campus_locations.size(); i++) {
            Node node;
            node.id = i;
            node.name = campus_locations[i].name;
            node.lat = campus_locations[i].lat;
            node.lon = campus_locations[i].lon;
            nodes.push_back(node);
        }
        
        // Build distance matrix and connections
        distances.resize(nodes.size(), vector<double>(nodes.size(), 0));
        
        for (int i = 0; i < nodes.size(); i++) {
            for (int j = 0; j < nodes.size(); j++) {
                if (i != j) {
                    double dist = haversine(nodes[i].lat, nodes[i].lon, nodes[j].lat, nodes[j].lon);
                    distances[i][j] = dist;
                    
                    // Connect nearby nodes (within 400m)
                    if (dist < 400) {
                        nodes[i].connections.push_back(j);
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
        vector<double> dist(nodes.size(), numeric_limits<double>::infinity());
        vector<int> parent(nodes.size(), -1);
        vector<bool> visited(nodes.size(), false);
        
        dist[start] = 0;
        
        for (int count = 0; count < nodes.size() - 1; count++) {
            int u = -1;
            for (int v = 0; v < nodes.size(); v++) {
                if (!visited[v] && (u == -1 || dist[v] < dist[u])) {
                    u = v;
                }
            }
            
            visited[u] = true;
            
            for (int v : nodes[u].connections) {
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
    
    vector<RouteStep> generateDetailedSteps(const vector<int>& path) {
        vector<RouteStep> steps;
        
        if (path.size() < 2) return steps;
        
        for (int i = 0; i < path.size() - 1; i++) {
            RouteStep step;
            step.distance = distances[path[i]][path[i + 1]];
            step.lat = nodes[path[i + 1]].lat;
            step.lon = nodes[path[i + 1]].lon;
            
            // Calculate direction
            double bearing = calculateBearing(
                nodes[path[i]].lat, nodes[path[i]].lon,
                nodes[path[i + 1]].lat, nodes[path[i + 1]].lon
            );
            
            step.direction = bearingToDirection(bearing);
            
            if (i == 0) {
                step.instruction = "Head " + step.direction + " towards " + nodes[path[i + 1]].name;
            } else if (i == path.size() - 2) {
                step.instruction = "Arrive at " + nodes[path[i + 1]].name;
            } else {
                step.instruction = "Continue " + step.direction + " for " + 
                                 to_string((int)step.distance) + " meters to " + nodes[path[i + 1]].name;
            }
            
            steps.push_back(step);
        }
        
        return steps;
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
        if (bearing >= 337.5 || bearing < 22.5) return "north";
        else if (bearing >= 22.5 && bearing < 67.5) return "northeast";
        else if (bearing >= 67.5 && bearing < 112.5) return "east";
        else if (bearing >= 112.5 && bearing < 157.5) return "southeast";
        else if (bearing >= 157.5 && bearing < 202.5) return "south";
        else if (bearing >= 202.5 && bearing < 247.5) return "southwest";
        else if (bearing >= 247.5 && bearing < 292.5) return "west";
        else return "northwest";
    }
    
    int findLocationByName(const string& name) {
        string lowerName = name;
        transform(lowerName.begin(), lowerName.end(), lowerName.begin(), ::tolower);
        
        // Exact match first
        for (int i = 0; i < campus_locations.size(); i++) {
            string locName = campus_locations[i].name;
            transform(locName.begin(), locName.end(), locName.begin(), ::tolower);
            if (locName == lowerName) return i;
        }
        
        // Partial match
        for (int i = 0; i < campus_locations.size(); i++) {
            string locName = campus_locations[i].name;
            transform(locName.begin(), locName.end(), locName.begin(), ::tolower);
            if (locName.find(lowerName) != string::npos || lowerName.find(locName) != string::npos) {
                return i;
            }
            
            // Check aliases
            for (const string& alias : campus_locations[i].aliases) {
                if (alias.find(lowerName) != string::npos || lowerName.find(alias) != string::npos) {
                    return i;
                }
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
                {"type", campus_locations[i].type},
                {"description", campus_locations[i].description}
            });
        }
        return locations;
    }
    
    json calculateRoute(double startLat, double startLon, const string& endName) {
        json result;
        
        // Find nearest start location
        int startId = -1;
        double minDist = numeric_limits<double>::infinity();
        for (int i = 0; i < nodes.size(); i++) {
            double dist = haversine(startLat, startLon, nodes[i].lat, nodes[i].lon);
            if (dist < minDist) {
                minDist = dist;
                startId = i;
            }
        }
        
        // Find end location
        int endId = findLocationByName(endName);
        
        if (startId == -1 || endId == -1) {
            result["success"] = false;
            result["error"] = "Location not found";
            return result;
        }
        
        // Find path
        vector<int> path = findShortestPath(startId, endId);
        
        if (path.empty()) {
            result["success"] = false;
            result["error"] = "No route found";
            return result;
        }
        
        // Generate detailed steps
        vector<RouteStep> steps = generateDetailedSteps(path);
        
        // Build response
        json pathJson = json::array();
        json stepsJson = json::array();
        double totalDistance = 0;
        
        // Add starting point
        pathJson.push_back({
            {"name", "Starting Point"},
            {"lat", startLat},
            {"lon", startLon}
        });
        
        for (int nodeId : path) {
            pathJson.push_back({
                {"name", nodes[nodeId].name},
                {"lat", nodes[nodeId].lat},
                {"lon", nodes[nodeId].lon}
            });
        }
        
        for (const RouteStep& step : steps) {
            stepsJson.push_back({
                {"instruction", step.instruction},
                {"distance", (int)step.distance},
                {"direction", step.direction},
                {"lat", step.lat},
                {"lon", step.lon}
            });
            totalDistance += step.distance;
        }
        
        result["success"] = true;
        result["path"] = pathJson;
        result["steps"] = stepsJson;
        result["total_distance"] = (int)totalDistance;
        result["start_location"] = nodes[startId].name;
        result["end_location"] = campus_locations[endId].name;
        
        return result;
    }
};

// Global navigation system
NavigationSystem navSystem;

// File reading utility
string readFile(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) return "";
    return string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
}

int main() {
    SimpleServer server;
    
    cout << "Enhanced NED Campus Navigator Server Starting..." << endl;
    
    // Serve static files
    server.Get("/", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        string content = readFile("enhanced_navigator.html");
        if (!content.empty()) {
            res.set_content(content, "text/html");
        } else {
            res.status = 404;
            res.body = "File not found";
        }
    });
    
    // API: Get all locations
    server.Get("/api/locations", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        json locations = navSystem.getLocationsJson();
        res.set_content(locations.dump(), "application/json");
    });
    
    // API: Calculate route from GPS coordinates
    server.Post("/api/route", [](const SimpleServer::Request& req, SimpleServer::Response& res) {
        try {
            json requestData = json::parse(req.body);
            
            double startLat = requestData["start_lat"];
            double startLon = requestData["start_lon"];
            string endName = requestData["destination"];
            
            json result = navSystem.calculateRoute(startLat, startLon, endName);
            res.set_content(result.dump(), "application/json");
            
        } catch (const exception& e) {
            json error = {
                {"success", false},
                {"error", "Invalid request format"}
            };
            res.set_content(error.dump(), "application/json");
        }
    });
    
    // API: Find location by name (for voice commands)
    server.Post("/api/find-location", [](const SimpleServer::Request& req, SimpleServer::Response& res) {
        try {
            json requestData = json::parse(req.body);
            string query = requestData["query"];
            
            int locationId = navSystem.findLocationByName(query);
            
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
                result["error"] = "Location not found";
            }
            
            res.set_content(result.dump(), "application/json");
            
        } catch (const exception& e) {
            json error = {{"success", false}, {"error", "Invalid request"}};
            res.set_content(error.dump(), "application/json");
        }
    });
    
    // Handle CORS preflight
    server.Options(".*", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        // CORS headers are already set in the server
    });
    
    cout << "Server running on http://localhost:8080" << endl;
    cout << "Enhanced features:" << endl;
    cout << "- GPS Integration" << endl;
    cout << "- Voice Commands" << endl;
    cout << "- Realistic Routing" << endl;
    cout << "- Step-by-step Navigation" << endl;
    cout << endl;
    cout << "Open http://localhost:8080 in your browser" << endl;
    
    server.listen("localhost", 8080);
    
    return 0;
}