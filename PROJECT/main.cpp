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

// GLOBAL PREDEFINED LOCATIONS
map<pair<double, double>, string> predefined_locations = {
    {{24.9328525, 67.1099341}, "NED University Admin Block"},
    {{24.9335294, 67.1110976}, "NED University Library"},
    {{24.9312853, 67.1125927}, "Urban Infrastructure Engineering Department"},
    {{24.9313328, 67.1126852}, "Civil Engineering Class Rooms"},
    {{24.9318691, 67.1135877}, "Survey Lab"},
    {{24.9313470, 67.1139814}, "CSIT Labs"},
    {{24.9313470, 67.1139814}, "CSIT Department Entrance"},
    {{24.9313470, 67.1139814}, "CSIT Offices"},
    {{24.9313470, 67.1139814}, "Computer Science Department"},
    {{24.9349057, 67.1107172}, "Polymer and Petrochemical Engineering"},
    {{24.9344365, 67.1109842}, "Mosque"},
    {{24.9330082, 67.1099861}, "NED Circular Road"},
    {{24.9312847, 67.1147127}, "Street 1"},
    {{24.9299167, 67.1156389}, "NED University Main Gate"},
    {{24.9325, 67.1139}, "Ring Street"},
    {{24.9325, 67.1138}, "Road 1"},
    {{24.9322, 67.1142}, "Road 2"},
    {{24.9335, 67.1155}, "NED Ground Road"},
    {{24.9335, 67.1150}, "Ground Road"},
    {{24.9325, 67.1164}, "Basketball Court"},
    {{24.9326117, 67.1163778}, "Tennis Court"},
    {{24.9328692, 67.1159699}, "Futsal Court"},
    {{24.9322537, 67.1167090}, "Athletics Track"},
    {{24.9315048, 67.1166501}, "Football Ground"},
    {{24.9309674, 67.1157977}, "Hockey Ground"},
    {{24.9322352, 67.1154004}, "Cricket Ground"},
    {{24.9302112, 67.1138518}, "Convocation Ground"},
    {{24.9307957, 67.1131117}, "Civil AV Hall"},
    {{24.9310011, 67.1129066}, "Civil Lecture Hall"},
    {{24.9320192, 67.1125931}, "Main Auditorium"},
    {{24.9317433, 67.1128339}, "Fountain Area"},
    {{24.9326224, 67.1144837}, "DMS Cafeteria"},
    {{24.9325, 67.1137}, "Meezan Bank ATM"},
    {{24.9330, 67.1139}, "Girls Gym"},
    {{24.9332, 67.1139}, "Boys Gym"},
    {{24.92896, 67.11352}, "NED Visitor Gate"},
    {{24.9296, 67.1129}, "National Incubation Centre"},
    {{24.9302, 67.1122}, "NED Service Department"},
    {{24.9304, 67.1122}, "SFC Stationary Store"},
    {{24.9304, 67.1120}, "SFC Canteen"},
    {{24.9308, 67.1124}, "Urban Lawn"},
    {{24.9314, 67.1119}, "Mechanical Lawn"},
    {{24.9314, 67.1121}, "NED Staff Centre"},
    {{24.9315, 67.1115}, "Mech Corner Cafe"},
    {{24.9321, 67.1109}, "NED Medical Centre"},
    {{24.9326, 67.1112}, "STEM Centre"},
    {{24.9332, 67.11083}, "Library Lawn"},
    {{24.93245, 67.1106}, "Dean Civil Engineering Office"},
    {{24.9327, 67.1104}, "NED White House"},
    {{24.93358, 67.10963}, "Transport Section"},
    {{24.9309500, 67.1139000}, "Mathematics Department"},
    {{24.9317800, 67.1120400}, "Mechanical Engineering Department"},
    {{24.9345252, 67.1125900}, "Environmental Engineering"},
    {{24.9326500, 67.1124000}, "Electrical Engineering Department"}
};

struct Node {
    string name;
    double lat, lon;
};

struct Edge {
    int to;
    double weight;
    bool open;
};

const double INF = 1e9;

// Core navigation functions
double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

vector<int> dijkstra(int start, int end, const vector<vector<Edge>>& graph) {
    int n = graph.size();
    vector<double> dist(n, INF);
    vector<int> parent(n, -1);
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;
    
    dist[start] = 0;
    pq.push({0, start});
    
    while (!pq.empty()) {
        double d = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        
        if (d > dist[u]) continue;
        
        for (const auto& edge : graph[u]) {
            if (!edge.open) continue;
            
            int v = edge.to;
            double weight = edge.weight;
            
            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                parent[v] = u;
                pq.push({dist[v], v});
            }
        }
    }
    
    vector<int> path;
    if (dist[end] == INF) return path;
    
    int current = end;
    while (current != -1) {
        path.push_back(current);
        current = parent[current];
    }
    reverse(path.begin(), path.end());
    return path;
}

// Global data structures
unordered_map<int, Node> nodes;
vector<vector<Edge>> graph;

void initialize_graph() {
    int node_id = 0;
    for (const auto& loc : predefined_locations) {
        nodes[node_id] = {loc.second, loc.first.first, loc.first.second};
        node_id++;
    }
    
    graph.resize(nodes.size());
    
    // Connect nearby locations
    for (const auto& node1 : nodes) {
        for (const auto& node2 : nodes) {
            if (node1.first != node2.first) {
                double dist = haversine(node1.second.lat, node1.second.lon,
                                      node2.second.lat, node2.second.lon);
                if (dist < 300) { // Connect if within 300 meters
                    graph[node1.first].push_back({node2.first, dist, true});
                }
            }
        }
    }
}

// API endpoint functions
json get_locations() {
    json locations = json::array();
    for (const auto& node : nodes) {
        locations.push_back({
            {"id", node.first},
            {"name", node.second.name},
            {"lat", node.second.lat},
            {"lon", node.second.lon}
        });
    }
    return locations;
}

json find_route(const string& start_name, const string& end_name) {
    json result;
    
    int start_id = -1, end_id = -1;
    for (const auto& node : nodes) {
        if (node.second.name == start_name) start_id = node.first;
        if (node.second.name == end_name) end_id = node.first;
    }
    
    if (start_id == -1 || end_id == -1) {
        result["success"] = false;
        result["error"] = "Location not found";
        return result;
    }
    
    vector<int> path = dijkstra(start_id, end_id, graph);
    
    if (path.empty()) {
        result["success"] = false;
        result["error"] = "No route found";
        return result;
    }
    
    json path_json = json::array();
    json steps_json = json::array();
    double total_distance = 0;
    
    for (int i = 0; i < path.size(); i++) {
        int node_id = path[i];
        path_json.push_back({
            {"name", nodes[node_id].name},
            {"lat", nodes[node_id].lat},
            {"lon", nodes[node_id].lon}
        });
        
        if (i > 0) {
            double dist = haversine(nodes[path[i-1]].lat, nodes[path[i-1]].lon,
                                  nodes[node_id].lat, nodes[node_id].lon);
            total_distance += dist;
            
            steps_json.push_back({
                {"instruction", "Continue to " + nodes[node_id].name},
                {"distance", (int)dist}
            });
        }
    }
    
    result["success"] = true;
    result["path"] = path_json;
    result["steps"] = steps_json;
    result["total_distance"] = (int)total_distance;
    
    return result;
}

string read_file(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) return "";
    return string((istreambuf_iterator<char>(file)), istreambuf_iterator<char>());
}

int main() {
    initialize_graph();
    
    SimpleServer server;
    
    // Serve static files
    server.Get("/", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        string content = read_file("index.html");
        if (!content.empty()) {
            res.set_content(content, "text/html");
        } else {
            res.status = 404;
        }
    });
    
    server.Get("/style.css", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        string content = read_file("style.css");
        if (!content.empty()) {
            res.set_content(content, "text/css");
        } else {
            res.status = 404;
        }
    });
    
    server.Get("/script.js", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        string content = read_file("script.js");
        if (!content.empty()) {
            res.set_content(content, "text/javascript");
        } else {
            res.status = 404;
        }
    });
    
    // API endpoints
    server.Get("/api/locations", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        json locations = get_locations();
        res.set_content(locations.dump(), "application/json");
    });
    
    server.Post("/api/route", [](const SimpleServer::Request& req, SimpleServer::Response& res) {
        try {
            json request_json = json::parse(req.body);
            string start = request_json["start"];
            string end = request_json["end"];
            
            json result = find_route(start, end);
            res.set_content(result.dump(), "application/json");
        } catch (...) {
            json error = {{"success", false}, {"error", "Invalid request"}};
            res.set_content(error.dump(), "application/json");
        }
    });
    
    server.Options(".*", [](const SimpleServer::Request&, SimpleServer::Response& res) {
        return;
    });
    
    cout << "NED Campus Navigator Server starting on http://localhost:8080" << endl;
    cout << "Open your browser and navigate to http://localhost:8080" << endl;
    
    server.listen("localhost", 8080);
    
    return 0;
}