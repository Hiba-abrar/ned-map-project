#include <iostream>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <set>
#include <limits>

using namespace std;

map<int, pair<double, double>> roadNodes = {
    {1, {24.9330, 67.1100}}, {2, {24.9328, 67.1110}}, {3, {24.9325, 67.1120}}, {4, {24.9320, 67.1130}},
    {5, {24.9315, 67.1140}}, {6, {24.9310, 67.1145}}, {7, {24.9305, 67.1140}}, {8, {24.9305, 67.1130}},
    {9, {24.9310, 67.1120}}, {10, {24.9315, 67.1110}}, {11, {24.9320, 67.1115}}, {12, {24.9318, 67.1125}},
    {13, {24.9315, 67.1130}}, {14, {24.9312, 67.1135}}, {15, {24.9314, 67.1140}}, {16, {24.9311, 67.1138}},
    {17, {24.9309, 67.1142}}, {18, {24.9325, 67.1150}}, {19, {24.9330, 67.1155}}, {20, {24.9324, 67.1165}},
    {21, {24.9326, 67.1167}}, {22, {24.9323, 67.1142}}, {23, {24.9323, 67.1114}}, {24, {24.9312, 67.1125}},
    {25, {24.9308, 67.1138}}, {26, {24.9332, 67.1102}}, {27, {24.9334, 67.1108}}, {28, {24.9316, 67.1148}},
    {29, {24.9318, 67.1152}}, {30, {24.9322, 67.1158}}, {31, {24.9328, 67.1162}}, {32, {24.9306, 67.1134}},
    {33, {24.9308, 67.1128}}, {34, {24.9313, 67.1122}}, {35, {24.9317, 67.1118}}, {36, {24.9321, 67.1112}},
    {37, {24.9325, 67.1106}}, {38, {24.9329, 67.1104}}, {39, {24.9333, 67.1110}}, {40, {24.9327, 67.1116}},
    {41, {24.9319, 67.1134}}, {42, {24.9311, 67.1142}}, {43, {24.9307, 67.1146}}, {44, {24.9303, 67.1138}},
    {45, {24.9301, 67.1132}}, {46, {24.9299, 67.1126}}, {47, {24.9297, 67.1120}}, {48, {24.9295, 67.1114}},
    {49, {24.9293, 67.1108}}, {50, {24.9291, 67.1102}}, {51, {24.9289, 67.1096}}, {52, {24.9287, 67.1090}}
};

map<string, int> locationToNode = {
    {"NED University Admin Block", 1}, {"NED University Library", 2}, {"CSIT Labs", 15}, {"NED University Main Gate", 4},
    {"Basketball Court", 20}, {"Football Ground", 21}, {"DMS Cafeteria", 22}, {"NED Medical Centre", 8},
    {"Mosque", 2}, {"Main Auditorium", 10}, {"Survey Lab", 7}, {"NPO Circuit", 17}, {"Urban Infrastructure Engineering Department", 24},
    {"Civil Engineering Class Rooms", 24}, {"Fire Lab", 23}, {"CSIT Department Entrance", 15}, {"CSIT Offices", 16},
    {"Tennis Court", 21}, {"Convocation Ground", 25}, {"Civil AV Hall", 8}, {"Mathematics Department", 14},
    {"Meezan Bank ATM", 22}, {"Street 1", 6}, {"Physics Department", 26}, {"Chemistry Department", 27},
    {"Electrical Engineering Department", 28}, {"Mechanical Engineering Department", 29}, {"Computer Systems Engineering Department", 30},
    {"Software Engineering Department", 31}, {"Industrial Engineering Department", 32}, {"Petroleum Engineering Department", 33},
    {"Materials Engineering Department", 34}, {"Biomedical Engineering Department", 35}, {"Architecture Department", 36},
    {"City Planning Department", 37}, {"Economics Department", 38}, {"Management Sciences Department", 39},
    {"English Department", 40}, {"Urdu Department", 41}, {"Islamic Studies Department", 42}, {"Pakistan Studies Department", 43},
    {"Student Affairs Office", 44}, {"Registrar Office", 45}, {"Finance Office", 46}, {"HR Department", 47},
    {"IT Services", 48}, {"Security Office", 49}, {"Maintenance Office", 50}, {"Parking Area A", 51},
    {"Parking Area B", 52}, {"Canteen", 22}, {"Bookstore", 23}
};

map<int, vector<pair<int, double>>> roadGraph;

struct NavigationStep {
    pair<double, double> location;
    string instruction;
    double distance_to_next;
    bool is_turn_point;
    string direction;
    string maneuver_type;
};

double haversine(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) + cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) * sin(dLon/2) * sin(dLon/2);
    return R * 2 * atan2(sqrt(a), sqrt(1-a));
}

double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
    double bearing = atan2(y, x) * 180.0 / M_PI;
    return fmod((bearing + 360.0), 360.0);
}

string get_turn_direction(double bearing1, double bearing2) {
    double diff = bearing2 - bearing1;
    if (diff > 180) diff -= 360;
    if (diff < -180) diff += 360;
    if (abs(diff) < 30) return "straight";
    return diff > 0 ? "right" : "left";
}

void initialize_road_graph() {
    for (int i = 1; i <= 52; i++) {
        roadGraph[i] = vector<pair<int, double>>();
    }
    
    for (int i = 1; i <= 10; i++) {
        int next = (i == 10) ? 1 : i + 1;
        double dist = haversine(roadNodes[i].first, roadNodes[i].second, roadNodes[next].first, roadNodes[next].second);
        roadGraph[i].push_back({next, dist});
        roadGraph[next].push_back({i, dist});
    }
    
    for (int i = 11; i <= 25; i++) {
        for (int j = 1; j <= 10; j++) {
            double dist = haversine(roadNodes[i].first, roadNodes[i].second, roadNodes[j].first, roadNodes[j].second);
            if (dist < 100) {
                roadGraph[i].push_back({j, dist});
                roadGraph[j].push_back({i, dist});
            }
        }
    }
    
    for (int i = 26; i <= 52; i++) {
        for (int j = 1; j <= 25; j++) {
            double dist = haversine(roadNodes[i].first, roadNodes[i].second, roadNodes[j].first, roadNodes[j].second);
            if (dist < 80) {
                roadGraph[i].push_back({j, dist});
                roadGraph[j].push_back({i, dist});
            }
        }
    }
}

vector<int> dijkstra_shortest_path(int start_node, int end_node) {
    map<int, double> distances;
    map<int, int> previous;
    set<int> unvisited;

    for (const auto &node : roadGraph) {
        distances[node.first] = numeric_limits<double>::infinity();
        unvisited.insert(node.first);
    }
    distances[start_node] = 0;

    while (!unvisited.empty()) {
        int current = -1;
        double min_dist = numeric_limits<double>::infinity();

        for (int node : unvisited) {
            if (distances[node] < min_dist) {
                min_dist = distances[node];
                current = node;
            }
        }

        if (current == -1 || current == end_node) break;
        unvisited.erase(current);

        for (const auto &neighbor : roadGraph[current]) {
            int next_node = neighbor.first;
            double edge_weight = neighbor.second;
            double alt = distances[current] + edge_weight;

            if (alt < distances[next_node]) {
                distances[next_node] = alt;
                previous[next_node] = current;
            }
        }
    }

    vector<int> path;
    int current = end_node;
    while (current != start_node && previous.find(current) != previous.end()) {
        path.push_back(current);
        current = previous[current];
    }
    path.push_back(start_node);
    reverse(path.begin(), path.end());
    return path;
}

string getLocations() {
    stringstream json;
    json << "{\"locations\":[";
    bool first = true;
    for (const auto& loc : locationToNode) {
        if (!first) json << ",";
        json << "{\"name\":\"" << loc.first << "\",";
        auto coords = roadNodes[loc.second];
        json << "\"lat\":" << coords.first << ",";
        json << "\"lon\":" << coords.second << "}";
        first = false;
    }
    json << "]}";
    return json.str();
}

string calculateRoute(const string& start, const string& end) {
    if (locationToNode.find(start) == locationToNode.end() || 
        locationToNode.find(end) == locationToNode.end()) {
        return "{\"error\":\"Location not found\"}";
    }
    
    int startNode = locationToNode[start];
    int endNode = locationToNode[end];
    
    vector<int> path = dijkstra_shortest_path(startNode, endNode);
    
    if (path.empty() || path.size() < 2) {
        return "{\"error\":\"No route found\"}";
    }
    
    vector<NavigationStep> route;
    double total_distance = 0;
    
    for (size_t i = 0; i < path.size(); i++) {
        NavigationStep step;
        int node_id = path[i];
        step.location = roadNodes[node_id];
        
        if (i == 0) {
            step.instruction = "Start from " + start;
            step.direction = "straight";
            step.is_turn_point = false;
            step.maneuver_type = "depart";
        } else if (i == path.size() - 1) {
            step.instruction = "Arrive at " + end;
            step.direction = "straight";
            step.is_turn_point = false;
            step.maneuver_type = "arrive";
        } else {
            double bearing1 = calculate_bearing(
                roadNodes[path[i-1]].first, roadNodes[path[i-1]].second,
                roadNodes[path[i]].first, roadNodes[path[i]].second);
            double bearing2 = calculate_bearing(
                roadNodes[path[i]].first, roadNodes[path[i]].second,
                roadNodes[path[i+1]].first, roadNodes[path[i+1]].second);
            
            step.direction = get_turn_direction(bearing1, bearing2);
            step.is_turn_point = (step.direction != "straight");
            
            if (step.is_turn_point) {
                step.instruction = "Turn " + step.direction;
                step.maneuver_type = "turn";
            } else {
                double distance_ahead = haversine(
                    roadNodes[path[i]].first, roadNodes[path[i]].second,
                    roadNodes[path[i+1]].first, roadNodes[path[i+1]].second);
                if (distance_ahead > 50) {
                    step.instruction = "Walk straight for " + to_string(int(distance_ahead)) + " meters";
                } else {
                    step.instruction = "Continue straight";
                }
                step.maneuver_type = "continue";
            }
        }
        
        if (i < path.size() - 1) {
            step.distance_to_next = haversine(
                roadNodes[path[i]].first, roadNodes[path[i]].second,
                roadNodes[path[i+1]].first, roadNodes[path[i+1]].second);
            total_distance += step.distance_to_next;
        } else {
            step.distance_to_next = 0;
        }
        
        route.push_back(step);
    }
    
    stringstream json;
    json << "{\"route\":[";
    for (size_t i = 0; i < route.size(); i++) {
        json << "{";
        json << "\"lat\":" << route[i].location.first << ",";
        json << "\"lon\":" << route[i].location.second << ",";
        json << "\"instruction\":\"" << route[i].instruction << "\",";
        json << "\"distance\":" << route[i].distance_to_next << ",";
        json << "\"is_turn\":" << (route[i].is_turn_point ? "true" : "false") << ",";
        json << "\"direction\":\"" << route[i].direction << "\",";
        json << "\"maneuver\":\"" << route[i].maneuver_type << "\"";
        json << "}";
        if (i < route.size() - 1) json << ",";
    }
    json << "],\"total_distance\":" << total_distance << ",\"total_steps\":" << route.size() << "}";
    
    return json.str();
}

string handleRequest(const string& request) {
    string response = "HTTP/1.1 200 OK\r\n";
    response += "Access-Control-Allow-Origin: *\r\n";
    response += "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n";
    response += "Access-Control-Allow-Headers: Content-Type\r\n";
    response += "Content-Type: application/json\r\n\r\n";
    
    if (request.find("GET /locations") != string::npos) {
        response += getLocations();
    } else if (request.find("GET /route") != string::npos) {
        size_t startPos = request.find("start=");
        size_t endPos = request.find("end=");
        
        if (startPos != string::npos && endPos != string::npos) {
            startPos += 6;
            size_t startEnd = request.find("&", startPos);
            if (startEnd == string::npos) startEnd = request.find(" ", startPos);
            
            endPos += 4;
            size_t endEnd = request.find("&", endPos);
            if (endEnd == string::npos) endEnd = request.find(" ", endPos);
            
            string start = request.substr(startPos, startEnd - startPos);
            string end = request.substr(endPos, endEnd - endPos);
            
            replace(start.begin(), start.end(), '+', ' ');
            replace(end.begin(), end.end(), '+', ' ');
            
            size_t pos = 0;
            while ((pos = start.find("%20", pos)) != string::npos) {
                start.replace(pos, 3, " ");
                pos += 1;
            }
            pos = 0;
            while ((pos = end.find("%20", pos)) != string::npos) {
                end.replace(pos, 3, " ");
                pos += 1;
            }
            
            response += calculateRoute(start, end);
        } else {
            response += "{\"error\":\"Missing start or end parameter\"}";
        }
    } else {
        response += "{\"error\":\"Endpoint not found\"}";
    }
    
    return response;
}

void handleClient(int clientSocket) {
    char buffer[4096] = {0};
    read(clientSocket, buffer, 4096);
    
    string request(buffer);
    string response = handleRequest(request);
    
    send(clientSocket, response.c_str(), response.length(), 0);
    close(clientSocket);
}

int main() {
    initialize_road_graph();
    
    int serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        cerr << "Failed to create socket" << endl;
        return -1;
    }
    
    int opt = 1;
    setsockopt(serverSocket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    
    sockaddr_in serverAddr;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(8080);
    
    if (::bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        cerr << "Bind failed" << endl;
        return -1;
    }
    
    if (listen(serverSocket, 3) < 0) {
        cerr << "Listen failed" << endl;
        return -1;
    }
    
    cout << "🎤 Voice Navigation Server running on http://localhost:8080" << endl;
    cout << "📍 Loaded " << locationToNode.size() << " locations" << endl;
    
    while (true) {
        sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        int clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientLen);
        
        if (clientSocket < 0) {
            cerr << "Accept failed" << endl;
            continue;
        }
        
        thread clientThread(handleClient, clientSocket);
        clientThread.detach();
    }
    
    close(serverSocket);
    return 0;
}