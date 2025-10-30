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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using json = nlohmann::json;

// GLOBAL PREDEFINED LOCATIONS - Updated with verified Google Maps coordinates
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
    {{24.9326500, 67.1124000}, "Electrical Engineering Department"}};

struct Node
{
    string name;
    double lat, lon;
};

struct Edge
{
    int to;
    double weight;
    bool open;
};

const double INF = 1e9;

// Voice synthesis simulation
void speak(const string &text)
{
    cout << text << endl;
    this_thread::sleep_for(chrono::milliseconds(500));
}

// Wait for enter key
void wait_for_enter()
{
    speak("Press ENTER to continue");
    string input;
    getline(cin, input);
}

// Wait for numeric input
int get_numeric_input(int min_val, int max_val)
{
    int choice;
    while (true)
    {
        if (cin >> choice)
        {
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            if (choice >= min_val && choice <= max_val)
            {
                return choice;
            }
        }
        else
        {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
        }
        speak("Invalid input. Please enter a number between " + to_string(min_val) + " and " + to_string(max_val));
    }
}

double haversine(double lat1, double lon1, double lat2, double lon2)
{
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

// GPS simulation - find nearest location
string find_nearest_location(double user_lat, double user_lon, int &location_choice, const map<int, string> &menu_options, const map<int, int> &menu_to_nodeid, const unordered_map<int, Node> &nodes)
{
    double min_distance = INF;
    string nearest_location = "";
    int nearest_choice = -1;
    
    for (const auto &loc : predefined_locations)
    {
        double distance = haversine(user_lat, user_lon, loc.first.first, loc.first.second);
        if (distance < min_distance)
        {
            min_distance = distance;
            nearest_location = loc.second;
            
            // Find corresponding menu choice
            for (const auto &menu : menu_options)
            {
                if (menu.second == loc.second)
                {
                    nearest_choice = menu.first;
                    break;
                }
            }
        }
    }
    
    location_choice = nearest_choice;
    return nearest_location;
}

// Get GPS coordinates directly
bool get_gps_location(double &lat, double &lon)
{
    speak("Open simple_gps.html in your browser to get GPS coordinates, then paste them here.");
    speak("Or enter coordinates manually.");
    speak("Enter coordinates as: latitude,longitude (e.g., 24.9325,67.1139)");
    
    string input;
    getline(cin, input);
    
    size_t comma_pos = input.find(',');
    if (comma_pos != string::npos)
    {
        try {
            lat = stod(input.substr(0, comma_pos));
            lon = stod(input.substr(comma_pos + 1));
        } catch (...) {
            speak("Invalid format. Please try again.");
            return false;
        }
    }
    else
    {
        speak("Invalid format. Use: latitude,longitude");
        return false;
    }
    
    // Validate coordinates are within NED University bounds
    if (lat >= 24.928 && lat <= 24.940 && lon >= 67.105 && lon <= 67.120)
    {
        speak("GPS location set successfully!");
        return true;
    }
    else
    {
        speak("Location is outside NED University campus. Please check your coordinates.");
        return false;
    }
}

// Check if two nodes are connected via DFS
bool is_connected(int start, int end, const vector<vector<Edge>> &graph, vector<bool> &visited)
{
    if (start == end) return true;
    visited[start] = true;
    
    for (const auto &edge : graph[start])
    {
        if (edge.open && !visited[edge.to])
        {
            if (is_connected(edge.to, end, graph, visited))
                return true;
        }
    }
    return false;
}

// Ensure all predefined locations are connected
void ensure_all_locations_connected(unordered_map<int, Node> &nodes, vector<vector<Edge>> &graph)
{
    speak("Ensuring all locations are connected");
    
    // Find all predefined location node IDs
    vector<int> location_nodes;
    for (const auto &node : nodes)
    {
        for (const auto &predef : predefined_locations)
        {
            if (node.second.name == predef.second)
            {
                location_nodes.push_back(node.first);
                break;
            }
        }
    }
    
    // Connect isolated locations to nearest accessible nodes
    for (int i = 0; i < location_nodes.size(); i++)
    {
        int node_id = location_nodes[i];
        vector<bool> visited(graph.size(), false);
        bool has_connection = false;
        
        // Check if this node has any open connections
        for (const auto &edge : graph[node_id])
        {
            if (edge.open)
            {
                has_connection = true;
                break;
            }
        }
        
        if (!has_connection)
        {
            // Find nearest node and connect
            double min_dist = INF;
            int nearest_node = -1;
            
            for (const auto &other_node : nodes)
            {
                if (other_node.first != node_id)
                {
                    double dist = haversine(nodes[node_id].lat, nodes[node_id].lon,
                                          other_node.second.lat, other_node.second.lon);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        nearest_node = other_node.first;
                    }
                }
            }
            
            if (nearest_node != -1)
            {
                graph[node_id].push_back({nearest_node, min_dist, true});
                graph[nearest_node].push_back({node_id, min_dist, true});
            }
        }
    }
    
    // Ensure connectivity between all location pairs
    for (int i = 0; i < location_nodes.size(); i++)
    {
        for (int j = i + 1; j < location_nodes.size(); j++)
        {
            vector<bool> visited(graph.size(), false);
            if (!is_connected(location_nodes[i], location_nodes[j], graph, visited))
            {
                // Connect via shortest distance
                double dist = haversine(nodes[location_nodes[i]].lat, nodes[location_nodes[i]].lon,
                                      nodes[location_nodes[j]].lat, nodes[location_nodes[j]].lon);
                graph[location_nodes[i]].push_back({location_nodes[j], dist, true});
                graph[location_nodes[j]].push_back({location_nodes[i], dist, true});
            }
        }
    }
    
    speak("All locations are now connected");
}

string get_building_name(const json &properties)
{
    if (properties.contains("name"))
    {
        string name = properties["name"];
        if (name == "NED University (Admin Block)" ||
            name == "NED University (Library)" ||
            name.find("NED University") != string::npos)
        {
            return name;
        }
        if (name.find("Department") != string::npos ||
            name.find("Civil Engg") != string::npos ||
            name.find("Survey Lab") != string::npos ||
            name.find("Computer System") != string::npos ||
            name.find("CSIT") != string::npos)
        {
            return name;
        }
    }

    if (properties.contains("building"))
    {
        string building_type = properties["building"];
        if (building_type == "university")
            return "University Building";
        else if (building_type == "sports_centre")
            return "Sports Centre";
        else if (building_type == "mosque")
            return "Mosque";
        else if (building_type == "apartments")
            return "Apartment Building";
    }

    if (properties.contains("addr:housename"))
        return properties["addr:housename"];
    if (properties.contains("name"))
        return properties["name"];

    return "Unnamed Location";
}

double calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double dLon_rad = (lon2 - lon1) * M_PI / 180.0;

    double y = sin(dLon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon_rad);
    double bearing = atan2(y, x);

    bearing = bearing * 180.0 / M_PI;
    bearing = fmod(bearing + 360.0, 360.0);

    return bearing;
}

bool load_map(const string &filename,
              unordered_map<int, Node> &nodes,
              vector<vector<Edge>> &graph)
{
    ifstream in(filename);
    if (!in)
        return false;

    json geo;
    in >> geo;

    nodes.clear();
    graph.clear();

    double min_lat = 24.928;
    double max_lat = 24.940;
    double min_lon = 67.105;
    double max_lon = 67.120;

    int id_counter = 0;
    map<pair<double, double>, int> coord_to_id;

    speak("Processing map data");

    for (auto &feature : geo["features"])
    {
        string type = feature["geometry"]["type"];
        if (type != "LineString")
            continue;

        auto coords = feature["geometry"]["coordinates"];
        vector<pair<double, double>> points;

        for (auto &pt : coords)
        {
            double lon = pt[0];
            double lat = pt[1];
            if (lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon)
                points.push_back({lat, lon});
        }

        int prevId = -1;
        for (auto &p : points)
        {
            double lat = p.first, lon = p.second;
            pair<double, double> key = {lat, lon};

            int nodeId = -1;
            bool found_existing = false;

            if (coord_to_id.count(key))
            {
                nodeId = coord_to_id[key];
                found_existing = true;
            }

            if (!found_existing)
            {
                for (auto &existing : nodes)
                {
                    double dist = haversine(lat, lon, existing.second.lat, existing.second.lon);
                    if (dist < 2.0)
                    {
                        nodeId = existing.first;
                        found_existing = true;
                        break;
                    }
                }
            }

            if (!found_existing)
            {
                nodeId = id_counter++;
                coord_to_id[key] = nodeId;

                string node_name = "Path_Node_" + to_string(nodeId);

                double min_dist = INF;
                string closest_name = "";
                for (const auto &loc : predefined_locations)
                {
                    double dist = haversine(lat, lon, loc.first.first, loc.first.second);
                    if (dist < min_dist && dist < 20.0)
                    {
                        min_dist = dist;
                        closest_name = loc.second;
                    }
                }
                if (!closest_name.empty())
                    node_name = closest_name;

                nodes[nodeId] = {node_name, lat, lon};
                if ((int)graph.size() <= nodeId)
                    graph.resize(nodeId + 1);
            }

            if (prevId != -1)
            {
                double dist = haversine(nodes[prevId].lat, nodes[prevId].lon, lat, lon);
                if (dist < 500)
                {
                    graph[prevId].push_back({nodeId, dist, true});
                    graph[nodeId].push_back({prevId, dist, true});
                }
            }

            prevId = nodeId;
        }
    }

    map<pair<double, double>, int> predefined_location_ids;

    for (const auto &loc : predefined_locations)
    {
        double lat = loc.first.first, lon = loc.first.second;
        string name = loc.second;
        pair<double, double> key = {lat, lon};

        int buildingId = -1;
        bool exists = false;

        for (auto &node_pair : nodes)
        {
            double d = haversine(lat, lon, node_pair.second.lat, node_pair.second.lon);
            if (d < 15.0)
            {
                node_pair.second.name = name;
                buildingId = node_pair.first;
                exists = true;
                break;
            }
        }

        if (!exists)
        {
            buildingId = id_counter++;
            nodes[buildingId] = {name, lat, lon};
            coord_to_id[key] = buildingId;
            if ((int)graph.size() <= buildingId)
                graph.resize(buildingId + 1);
        }

        predefined_location_ids[key] = buildingId;

        double min_dist = INF;
        int nearest_node = -1;
        for (const auto &node_pair : nodes)
        {
            int nodeId = node_pair.first;
            const Node &node = node_pair.second;

            if (nodeId == buildingId)
                continue;

            double dist = haversine(lat, lon, node.lat, node.lon);

            if (dist < min_dist && dist < 50.0)
            {
                min_dist = dist;
                nearest_node = nodeId;
            }
        }

        if (nearest_node != -1 && min_dist > 2.0)
        {
            graph[buildingId].push_back({nearest_node, min_dist, true});
            graph[nearest_node].push_back({buildingId, min_dist, true});
        }
    }

    speak("Map loaded successfully with " + to_string(nodes.size()) + " locations");
    return true;
}

// Check if a node is closed (all edges are closed)
bool is_node_closed(int node_id, const vector<vector<Edge>> &graph)
{
    if (node_id < 0 || node_id >= (int)graph.size())
        return true;
    
    for (const auto &edge : graph[node_id])
    {
        if (edge.open)
            return false;
    }
    return true;
}

vector<double> dijkstra(int n, const vector<vector<Edge>> &graph, int src, vector<int> &parent)
{
    vector<double> dist(n, INF);
    parent.assign(n, -1);
    if (src < 0 || src >= n)
        return dist;
    dist[src] = 0;
    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    pq.push(make_pair(0, src));
    
    while (!pq.empty())
    {
        P top = pq.top();
        pq.pop();
        double d = top.first;
        int u = top.second;
        
        if (d != dist[u])
            continue;
        
        // COMPLETELY SKIP CLOSED NODES (except source and destination)
        if (u != src && is_node_closed(u, graph))
            continue;
            
        for (const auto &e : graph[u])
        {
            if (!e.open)
                continue;
                
            int v = e.to;
            
            // NEVER PROCESS CLOSED NODES - COMPLETE EXCLUSION
            if (v != src && is_node_closed(v, graph))
                continue;
                
            double nd = d + e.weight;
            if (nd < dist[v])
            {
                dist[v] = nd;
                parent[v] = u;
                // Only add to queue if destination is not closed
                if (!is_node_closed(v, graph) || v == src)
                {
                    pq.push(make_pair(nd, v));
                }
            }
        }
    }
    return dist;
}

// Set node open/closed status - completely blocks the node
void set_node_open(vector<vector<Edge>> &graph, int node_id, bool open)
{
    if (node_id < 0 || node_id >= (int)graph.size())
        return;
    
    // Set all edges from this node
    for (auto &edge : graph[node_id])
    {
        edge.open = open;
    }
    
    // Set all edges to this node
    for (int i = 0; i < (int)graph.size(); i++)
    {
        for (auto &edge : graph[i])
        {
            if (edge.to == node_id)
            {
                edge.open = open;
            }
        }
    }
}

// Set specific edge open/closed status
void set_edge_open(vector<vector<Edge>> &graph, int u, int v, bool open)
{
    if (u < 0 || v < 0 || u >= (int)graph.size() || v >= (int)graph.size())
        return;
    for (auto &e : graph[u])
        if (e.to == v)
            e.open = open;
    for (auto &e : graph[v])
        if (e.to == u)
            e.open = open;
}

double find_edge_weight(const vector<vector<Edge>> &graph, int u, int v)
{
    if (u < 0 || v < 0 || u >= (int)graph.size() || v >= (int)graph.size())
        return -1;
    for (const auto &e : graph[u])
        if (e.to == v)
            return e.weight;
    return -1;
}

vector<int> find_nodes_by_name(const unordered_map<int, Node> &nodes, const string &name)
{
    vector<int> result;
    for (const auto &node : nodes)
    {
        if (node.second.name.find(name) != string::npos)
        {
            result.push_back(node.first);
        }
    }
    return result;
}

// Convert meters to steps (average step length is 0.762 meters / 2.5 feet)
string format_steps(double meters)
{
    // Average step length for adults is approximately 0.762 meters (2.5 feet)
    const double STEP_LENGTH = 0.762;
    int steps = static_cast<int>(round(meters / STEP_LENGTH));

    if (steps == 1)
    {
        return "1 step";
    }
    else if (steps < 1000)
    {
        return to_string(steps) + " steps";
    }
    else
    {
        return to_string(steps) + " steps";
    }
}

// Keep original format_distance for total distance display
string format_distance(double meters)
{
    if (meters < 1.0)
        return to_string(int(meters * 100)) + " centimeters";
    else if (meters < 1000.0)
        return to_string(int(meters)) + " meters";
    else
        return to_string(int(meters / 1000.0)) + " kilometers and " + to_string(int(fmod(meters, 1000.0))) + " meters";
}

void list_locations_menu(const unordered_map<int, Node> &nodes)
{
    speak("Available locations at NED University");

    int counter = 1;
    for (const auto &predef : predefined_locations)
    {
        string name = predef.second;
        vector<int> node_ids = find_nodes_by_name(nodes, name);

        if (!node_ids.empty())
        {
            cout << counter << ") " << name << endl;
            counter++;
        }
    }
    speak("End of location list");
}

// Function to check if a node name is a Path_Node (unnamed node)
bool is_path_node(const string &node_name)
{
    return node_name.find("Path_Node_") != string::npos;
}

// Get real location name from coordinates
string get_real_location_name(double lat, double lon, const string &current_name)
{
    if (!is_path_node(current_name))
    {
        return current_name;
    }
    
    // Find nearest predefined location
    double min_dist = INF;
    string nearest_name = "pathway";
    
    for (const auto &loc : predefined_locations)
    {
        double dist = haversine(lat, lon, loc.first.first, loc.first.second);
        if (dist < min_dist && dist < 30.0) // Within 30 meters
        {
            min_dist = dist;
            nearest_name = loc.second;
        }
    }
    
    // If still generic, provide directional context
    if (nearest_name == "pathway")
    {
        if (lat > 24.935) nearest_name = "northern campus area";
        else if (lat < 24.930) nearest_name = "southern campus area";
        else if (lon > 67.115) nearest_name = "eastern campus area";
        else if (lon < 67.110) nearest_name = "western campus area";
        else nearest_name = "central campus area";
    }
    
    return nearest_name;
}

// Get direction instruction based on bearing change
string get_direction_instruction(double prev_bearing, double current_bearing, bool is_first_step)
{
    if (is_first_step)
    {
        return "Go straight ahead";
    }
    
    double turn = current_bearing - prev_bearing;
    if (turn < -180) turn += 360;
    if (turn > 180) turn -= 360;
    
    if (fabs(turn) < 15) return "Continue straight";
    else if (turn > 15 && turn < 45) return "Turn slightly right";
    else if (turn >= 45 && turn < 135) return "Turn right";
    else if (turn >= 135) return "Turn sharply right";
    else if (turn < -15 && turn > -45) return "Turn slightly left";
    else if (turn <= -45 && turn > -135) return "Turn left";
    else return "Turn sharply left";
}

void navigation_mode(const unordered_map<int, Node> &nodes, const vector<vector<Edge>> &graph)
{
    speak("Navigation mode");
    speak("Step 1 - Choose start location");

    int counter = 1;
    map<int, string> menu_options;
    map<int, int> menu_to_nodeid;

    for (const auto &predef : predefined_locations)
    {
        string name = predef.second;
        vector<int> node_ids = find_nodes_by_name(nodes, name);

        if (!node_ids.empty())
        {
            menu_options[counter] = name;
            menu_to_nodeid[counter] = node_ids[0];
            counter++;
        }
    }

    int total_locations = menu_options.size();
    int start_choice = -1;
    int dest_choice = -1;

    // STEP 1: Choose location method
    speak("How would you like to select your start location?");
    speak("1 Use GPS location");
    speak("2 Select from menu");
    int location_method = get_numeric_input(1, 2);
    
    if (location_method == 1) // GPS
    {
        double user_lat, user_lon;
        while (!get_gps_location(user_lat, user_lon))
        {
            speak("Please enter valid GPS coordinates within NED University campus.");
        }
        
        string nearest_location = find_nearest_location(user_lat, user_lon, start_choice, menu_options, menu_to_nodeid, nodes);
        speak("GPS detected your location near: " + nearest_location);
        speak("Using this as your start location.");
    }
    else // Manual menu selection
    {
        speak("Enter start location number (1-" + to_string(total_locations) + "):");
        start_choice = get_numeric_input(1, total_locations);
    }
    
    speak("Start location: " + menu_options[start_choice]);
    
    // Check if start location is CSIT-related and ask for floor
    string start_location_name = menu_options[start_choice];
    bool is_csit_location = (start_location_name.find("CSIT") != string::npos || 
                            start_location_name.find("Computer Science") != string::npos);
    
    if (is_csit_location)
    {
        speak("Which floor are you currently on?");
        speak("1 Ground Floor");
        speak("2 First Floor");
        speak("3 Second Floor");
        int floor_choice = get_numeric_input(1, 3);
        
        if (floor_choice == 2) // First Floor
        {
            speak("Go left, go to the left extreme of the floor. Following the wall, you will find stairs there. Use those stairs to come to the ground floor.");
        }
        else if (floor_choice == 3) // Second Floor
        {
            speak("Go left, go to the left extreme of the floor. Following the wall, you will find stairs there. Use those stairs to come to the ground floor.");
        }
        // If Ground Floor (choice == 1), continue normally
    }

    // STEP 2: Choose destination location
    speak("Enter destination location number (1-" + to_string(total_locations) + ")");
    while (dest_choice == -1)
    {
        int choice = get_numeric_input(1, total_locations);
        if (choice == start_choice)
        {
            speak("Destination cannot be same as start location. Please choose a different location.");
        }
        else
        {
            dest_choice = choice;
            speak("Destination: " + menu_options[dest_choice]);
        }
    }

    // Calculate route
    int start_id = menu_to_nodeid[start_choice];
    int dest_id = menu_to_nodeid[dest_choice];

    speak("Calculating route from " + menu_options[start_choice] + " to " + menu_options[dest_choice]);

    vector<int> parent;
    vector<double> dist = dijkstra((int)graph.size(), graph, start_id, parent);

    if (dist[dest_id] >= INF)
    {
        speak("No path found between these locations");
        return;
    }

    vector<int> path;
    for (int cur = dest_id; cur != -1; cur = parent[cur])
        path.push_back(cur);
    reverse(path.begin(), path.end());
    
    // TRIPLE CHECK: Validate path doesn't include ANY closed nodes
    bool path_has_closed_nodes = false;
    for (int i = 1; i < path.size() - 1; i++) // Skip start and end nodes
    {
        if (is_node_closed(path[i], graph))
        {
            speak("ERROR: Route includes closed location: " + nodes.at(path[i]).name);
            path_has_closed_nodes = true;
        }
    }
    
    if (path_has_closed_nodes)
    {
        speak("Route calculation failed: Path includes closed locations. Please try again.");
        return;
    }
    
    // FINAL CHECK: Verify all edges in path are open
    for (int i = 0; i < path.size() - 1; i++)
    {
        bool edge_found = false;
        for (const auto &edge : graph[path[i]])
        {
            if (edge.to == path[i + 1] && edge.open)
            {
                edge_found = true;
                break;
            }
        }
        if (!edge_found)
        {
            speak("ERROR: Route uses closed edge between " + nodes.at(path[i]).name + " and " + nodes.at(path[i + 1]).name);
            speak("Route calculation failed. Please try again.");
            return;
        }
    }

    speak("Route found. Total distance: " + format_distance(dist[dest_id]));
    speak("Press ENTER for navigation instructions");
    wait_for_enter();

    // Enhanced Navigation instructions
    double prevBearing = -1;
    for (size_t i = 0; i + 1 < path.size(); i++)
    {
        int u = path[i], v = path[i + 1];
        double w = find_edge_weight(graph, u, v);

        if (w > 0)
        {
            double bearing = calculate_bearing(
                nodes.at(u).lat, nodes.at(u).lon,
                nodes.at(v).lat, nodes.at(v).lon);

            // Get enhanced direction instruction
            string instruction = get_direction_instruction(prevBearing, bearing, prevBearing == -1);

            // Get real destination name with coordinate lookup
            string destination_name = get_real_location_name(
                nodes.at(v).lat, nodes.at(v).lon, nodes.at(v).name);

            // Enhanced step information with clear directions
            string step_info;
            if (i == 0) // First step
            {
                step_info = instruction + " for " + format_steps(w) + " towards " + destination_name;
            }
            else
            {
                step_info = instruction + " and walk " + format_steps(w) + " towards " + destination_name;
            }
            
            speak("Step " + to_string(i + 1) + " of " +
                  to_string(path.size() - 1) + ": " + step_info);

            if (i + 1 < path.size() - 1)
            {
                speak("Press N for next instruction, R to repeat");
                while (true)
                {
                    string nav_input;
                    getline(cin, nav_input);

                    if (nav_input.length() == 1)
                    {
                        char nav_key = tolower(nav_input[0]);
                        if (nav_key == 'n')
                        {
                            break;
                        }
                        else if (nav_key == 'r')
                        {
                            speak("Step " + to_string(i + 1) + " of " +
                                  to_string(path.size() - 1) + ": " + step_info);
                            continue;
                        }
                    }
                    speak("Please press N for next or R to repeat");
                }
            }
            else
            {
                speak("You have arrived at your destination: " + menu_options[dest_choice]);
            }

            prevBearing = bearing;
        }
    }
}

int main(int argc, char **argv)
{
    string filename = (argc > 1) ? argv[1] : "export.geojson";
    unordered_map<int, Node> nodes;
    vector<vector<Edge>> graph;

    if (!load_map(filename, nodes, graph))
    {
        speak("Failed to load map file");
        return 1;
    }

    ensure_all_locations_connected(nodes, graph);

    while (true)
    {
        speak("MENU");
        speak("1 List locations");
        speak("2 Navigate");
        speak("3 Path tools");
        speak("4 Reload map");
        speak("5 Exit");

        int choice = get_numeric_input(1, 5);

        if (choice == 1)
        {
            list_locations_menu(nodes);
            wait_for_enter();
        }
        else if (choice == 2)
        {
            navigation_mode(nodes, graph);
            wait_for_enter();
        }
        else if (choice == 3)
        {
            speak("Path tools menu:");
            speak("1 Close location");
            speak("2 Open location");
            speak("3 Close path between locations");
            speak("4 Open path between locations");
            speak("5 Back to main menu");
            int tool_choice = get_numeric_input(1, 5);
            
            // Build menu options for path tools
            map<int, string> tool_menu_options;
            map<int, int> tool_menu_to_nodeid;
            int tool_counter = 1;
            
            for (const auto &predef : predefined_locations)
            {
                string name = predef.second;
                vector<int> node_ids = find_nodes_by_name(nodes, name);
                if (!node_ids.empty())
                {
                    tool_menu_options[tool_counter] = name;
                    tool_menu_to_nodeid[tool_counter] = node_ids[0];
                    tool_counter++;
                }
            }
            
            if (tool_choice == 1 || tool_choice == 2) // Location operations
            {
                speak("Enter location number (1-" + to_string(tool_menu_options.size()) + "):");
                int menu_choice = get_numeric_input(1, tool_menu_options.size());
                
                int node_id = tool_menu_to_nodeid[menu_choice];
                set_node_open(graph, node_id, tool_choice == 2);
                speak((tool_choice == 2 ? "Location opened: " : "Location closed: ") + tool_menu_options[menu_choice]);
            }
            else if (tool_choice == 3 || tool_choice == 4) // Path operations
            {
                speak("Enter first location number (1-" + to_string(tool_menu_options.size()) + "):");
                int menu1 = get_numeric_input(1, tool_menu_options.size());
                speak("Enter second location number (1-" + to_string(tool_menu_options.size()) + "):");
                int menu2 = get_numeric_input(1, tool_menu_options.size());
                
                int node1 = tool_menu_to_nodeid[menu1];
                int node2 = tool_menu_to_nodeid[menu2];
                set_edge_open(graph, node1, node2, tool_choice == 4);
                speak((tool_choice == 4 ? "Path opened between: " : "Path closed between: ") + 
                      tool_menu_options[menu1] + " and " + tool_menu_options[menu2]);
            }
        }
        else if (choice == 4)
        {
            if (load_map(filename, nodes, graph))
            {
                ensure_all_locations_connected(nodes, graph);
                speak("Map reloaded successfully");
            }
            else
            {
                speak("Map reload failed");
            }
        }
        else if (choice == 5)
        {
            speak("Thank you for using NED University Navigation System. Goodbye!");
            break;
        }
    }

    return 0;
}