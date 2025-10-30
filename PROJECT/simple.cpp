#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")

using namespace std;

struct Location {
    string name;
    double lat, lon;
};

vector<Location> locations = {
    {"NED University Main Gate", 24.9299167, 67.1156389},
    {"NED University Library", 24.9335294, 67.1110976},
    {"NED University Admin Block", 24.9328525, 67.1099341},
    {"CSIT Labs", 24.9313470, 67.1139814},
    {"Main Auditorium", 24.9320192, 67.1125931},
    {"DMS Cafeteria", 24.9326224, 67.1144837}
};

double distance(const Location& a, const Location& b) {
    double dx = (a.lat - b.lat) * 111000;
    double dy = (a.lon - b.lon) * 111000;
    return sqrt(dx*dx + dy*dy);
}

string findRoute(const string& start, const string& end) {
    Location* startLoc = nullptr;
    Location* endLoc = nullptr;
    
    for (auto& loc : locations) {
        if (loc.name == start) startLoc = &loc;
        if (loc.name == end) endLoc = &loc;
    }
    
    if (!startLoc || !endLoc) return "{\"success\":false,\"error\":\"Location not found\"}";
    
    double dist = distance(*startLoc, *endLoc);
    
    return "{\"success\":true,\"path\":[{\"name\":\"" + start + "\",\"lat\":" + 
           to_string(startLoc->lat) + ",\"lon\":" + to_string(startLoc->lon) + 
           "},{\"name\":\"" + end + "\",\"lat\":" + to_string(endLoc->lat) + 
           ",\"lon\":" + to_string(endLoc->lon) + "}],\"steps\":[{\"instruction\":\"Go to " + 
           end + "\",\"distance\":" + to_string((int)dist) + "}],\"total_distance\":" + 
           to_string((int)dist) + "}";
}

string getLocations() {
    string result = "[";
    for (int i = 0; i < locations.size(); i++) {
        if (i > 0) result += ",";
        result += "{\"id\":" + to_string(i) + ",\"name\":\"" + locations[i].name + 
                  "\",\"lat\":" + to_string(locations[i].lat) + 
                  ",\"lon\":" + to_string(locations[i].lon) + "}";
    }
    result += "]";
    return result;
}

void handleRequest(SOCKET client, const string& request) {
    string response;
    string headers = "HTTP/1.1 200 OK\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\n";
    
    if (request.find("GET /api/locations") != string::npos) {
        string body = getLocations();
        response = headers + "Content-Type: application/json\r\nContent-Length: " + to_string(body.length()) + "\r\n\r\n" + body;
    }
    else if (request.find("POST /api/route") != string::npos) {
        size_t bodyStart = request.find("\r\n\r\n");
        if (bodyStart != string::npos) {
            string body = request.substr(bodyStart + 4);
            size_t startPos = body.find("\"start\":\"") + 9;
            size_t startEnd = body.find("\"", startPos);
            size_t endPos = body.find("\"end\":\"") + 7;
            size_t endEnd = body.find("\"", endPos);
            
            if (startEnd != string::npos && endEnd != string::npos) {
                string start = body.substr(startPos, startEnd - startPos);
                string end = body.substr(endPos, endEnd - endPos);
                string result = findRoute(start, end);
                response = headers + "Content-Type: application/json\r\nContent-Length: " + to_string(result.length()) + "\r\n\r\n" + result;
            }
        }
    }
    else if (request.find("GET /") != string::npos) {
        string html = "<!DOCTYPE html><html><head><title>NED Navigator</title></head><body><h1>NED Campus Navigator</h1><p>API is running!</p><p>Use /api/locations and /api/route endpoints</p></body></html>";
        response = headers + "Content-Type: text/html\r\nContent-Length: " + to_string(html.length()) + "\r\n\r\n" + html;
    }
    
    if (response.empty()) {
        response = "HTTP/1.1 404 Not Found\r\n\r\n";
    }
    
    send(client, response.c_str(), response.length(), 0);
}

int main() {
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    
    SOCKET server = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(8080);
    
    bind(server, (sockaddr*)&addr, sizeof(addr));
    listen(server, 5);
    
    cout << "Server running on http://localhost:8080" << endl;
    
    while (true) {
        SOCKET client = accept(server, nullptr, nullptr);
        char buffer[4096] = {0};
        recv(client, buffer, sizeof(buffer), 0);
        
        handleRequest(client, string(buffer));
        closesocket(client);
    }
    
    closesocket(server);
    WSACleanup();
    return 0;
}