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
#include "json.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;
using json = nlohmann::json;

struct Node
{
    string name;
    double lat, lon; // Store original lat/lon
};

struct Edge
{
    int to;
    double weight; // Changed to double for more precision
    bool open;
};

const double INF = 1e9;

// ------------------- Haversine distance -------------------
double haversine(double lat1, double lon1, double lat2, double lon2)
{
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

// ------------------- Get name from GeoJSON properties -------------------
string get_building_name(const json &properties)
{
    // Check for specific NED University buildings first
    if (properties.contains("name"))
    {
        string name = properties["name"];
        // NED University specific buildings
        if (name == "NED University (Admin Block)" ||
            name == "NED University (Library)" ||
            name.find("NED University") != string::npos)
        {
            return name;
        }
        // Department buildings
        if (name.find("Department") != string::npos ||
            name.find("Civil Engg") != string::npos ||
            name.find("Survey Lab") != string::npos ||
            name.find("Computer System") != string::npos ||
            name.find("CSIT") != string::npos)
        {
            return name;
        }
    }

    // Check for building type with specific names
    if (properties.contains("building"))
    {
        string building_type = properties["building"];
        if (building_type == "university")
        {
            if (properties.contains("addr:housename"))
            {
                return properties["addr:housename"];
            }
            return "University Building";
        }
        else if (building_type == "sports_centre")
        {
            return "Sports Centre";
        }
        else if (building_type == "mosque")
        {
            return "Mosque";
        }
        else if (building_type == "apartments")
        {
            if (properties.contains("name"))
            {
                return properties["name"];
            }
            return "Apartment Building";
        }
    }

    // Check for address names
    if (properties.contains("addr:housename"))
    {
        return properties["addr:housename"];
    }

    // Road names
    if (properties.contains("highway"))
    {
        if (properties.contains("name"))
        {
            string road_name = properties["name"];
            if (road_name == "NED Circular Road" ||
                road_name == "Street 1" ||
                road_name.find("University Road") != string::npos)
            {
                return road_name;
            }
        }
    }

    // Generic fallbacks
    if (properties.contains("name"))
    {
        return properties["name"];
    }

    return "Unnamed Location";
}

// ------------------- Check if node is within NED University premises -------------------
bool is_within_ned_premises(const string &node_name, double lat, double lon)
{
    // Filter out unwanted roads
    if (node_name.find("Abul Hasan") != string::npos ||
        node_name.find("Isphani") != string::npos ||
        node_name.find("Johar Chowrangi") != string::npos ||
        node_name.find("johar chowrangi") != string::npos)
    {
        return false;
    }

    // NED University boundary
    double ned_min_lat = 24.928; // Southern boundary (expanded for Main Gate)
    double ned_max_lat = 24.940; // Northern boundary
    double ned_min_lon = 67.105; // Western boundary (Kept wide as requested)
    double ned_max_lon = 67.120; // Eastern boundary

    // Check if coordinates are within NED premises
    if (lat < ned_min_lat || lat > ned_max_lat || lon < ned_min_lon || lon > ned_max_lon)
    {
        return false;
    }

    // Additional filter for generic road nodes that might slip through
    if (node_name.find("_Road_Node") != string::npos ||
        node_name.find("_Node") != string::npos)
    {
        // Only keep NED Circular Road and Street 1
        if (node_name.find("NED Circular Road") == string::npos &&
            node_name.find("Street 1") == string::npos)
        {
            return false;
        }
    }

    return true;
}

// ------------------- Calculate bearing between two points -------------------
double calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
    double lat1_rad = lat1 * M_PI / 180.0;
    double lat2_rad = lat2 * M_PI / 180.0;
    double dLon_rad = (lon2 - lon1) * M_PI / 180.0;

    double y = sin(dLon_rad) * cos(lat2_rad);
    double x = cos(lat1_rad) * sin(lat2_rad) - sin(lat1_rad) * cos(lat2_rad) * cos(dLon_rad);
    double bearing = atan2(y, x);

    // Convert from radians to degrees and normalize to 0-360
    bearing = bearing * 180.0 / M_PI;
    bearing = fmod(bearing + 360.0, 360.0);

    return bearing;
}

// ------------------- Get direction from bearing -------------------
string get_direction(double bearing)
{
    if (bearing >= 337.5 || bearing < 22.5)
        return "North";
    if (bearing >= 22.5 && bearing < 67.5)
        return "North-East";
    if (bearing >= 67.5 && bearing < 112.5)
        return "East";
    if (bearing >= 112.5 && bearing < 157.5)
        return "South-East";
    if (bearing >= 157.5 && bearing < 202.5)
        return "South";
    if (bearing >= 202.5 && bearing < 247.5)
        return "South-West";
    if (bearing >= 247.5 && bearing < 292.5)
        return "West";
    if (bearing >= 292.5 && bearing < 337.5)
        return "North-West";
    return "North";
}


// ------------------- Load GeoJSON (UPDATED FOR TARGETED DELETION 455-467 AND NODE 465 ADDITION) -------------------
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

    double min_lat = 24.928; // Southern boundary
    double max_lat = 24.940; // Northern boundary
    double min_lon = 67.105; // Western boundary (Wide as requested)
    double max_lon = 67.120; // Eastern boundary

    int id_counter = 0;
    map<pair<double, double>, int> coord_to_id;
    
    // --- Predefined Locations (All Validated Names) ---
    map<pair<double, double>, string> predefined_locations = {
        // --- Main Academic & Administrative Buildings (Existing) ---
        {{24.9328525, 67.1099341}, "NED University (Admin Block)"},
        {{24.9335294, 67.1110976}, "NED University (Library)"},
        {{24.9312853, 67.1125927}, "Department of Urban & Infrastructure Engineering"},
        {{24.9313328, 67.1126852}, "Civil Engineering Class Rooms"},
        {{24.9318691, 67.1135877}, "Survey Lab"},
        {{24.9313470, 67.1139814}, "CSIT Labs"},
        {{24.9310000, 67.1140000}, "CSIT Department Corridor Entrance"},
        {{24.9311094, 67.1135478}, "CSIT Offices"},
        {{24.9349057, 67.1107172}, "Department of Polymer and Petrochemical Engineering"},
        {{24.9344365, 67.1109842}, "Mosque"},
        {{24.9330082, 67.1099861}, "NED Circular Road"},
        {{24.9312847, 67.1147127}, "Street 1"},
        {{24.9299167, 67.1156389}, "NED University Main Gate"},

        // --- Nearby Buildings & Roads (Existing) ---
        {{24.9325, 67.1139}, "Ring Street"},
        {{24.9325, 67.1138}, "Road 1 leading to Ring Street"},
        {{24.9322, 67.1142}, "Road 2 towards Ring Street"},
        {{24.9335, 67.1155}, "NED Ground Road"},
        {{24.9335, 67.1150}, "Ground Road"},

        // --- Sports & Outdoor Areas (Existing) ---
        {{24.9325, 67.1164}, "Basketball Court"},
        {{24.9326117, 67.1163778}, "Tennis Court"},
        {{24.9328692, 67.1159699}, "Futsal Court"},
        {{24.9322537, 67.1167090}, "Athletics Track"},
        {{24.9315048, 67.1166501}, "Football Ground"},
        {{24.9309674, 67.1157977}, "Hockey Ground"},
        {{24.9322352, 67.1154004}, "Cricket Ground"},
        {{24.9302112, 67.1138518}, "Convocation Ground"},
        {{24.9307957, 67.1131117}, "Civil AV Hall"},

        // --- Central Academic and Lecture Buildings (Existing) ---
        {{24.9310011, 67.1129066}, "Civil Lecture Hall"},
        // {{24.9309930, 67.1139407}, "Department of Mathematics"}, // REMOVED OLD ENTRY
        {{24.9320192, 67.1125931}, "Main Auditorium"},
        {{24.9317433, 67.1128339}, "Fountain Area"},

        // --- Additional Landmarks (Existing) ---
        {{24.9326224, 67.1144837}, "DMS Cafeteria"},
        {{24.9325, 67.1137}, "Meezan Bank ATM"},
        {{24.9330, 67.1139}, "Girls Gym"},
        {{24.9332, 67.1139}, "Boys Gym"},
        
        // --- NEW LOCATIONS ---
        {{24.92896, 67.11352}, "NED Visitor's Gate"},
        {{24.9296, 67.1129}, "National Incubation Centre Karachi"},
        {{24.9302, 67.1122}, "NED Service Department"},
        {{24.9304, 67.1122}, "SFC Stationary Store"},
        {{24.9304, 67.1120}, "SFC Canteen"},
        {{24.9308, 67.1124}, "Urban Lawn"},
        {{24.9314, 67.1119}, "NED Mechanical Lawn"},
        {{24.9314, 67.1121}, "NED Staff Centre"},
        {{24.9315, 67.1115}, "Mech Corner Cafe"},
        {{24.9321, 67.1109}, "NED Medical Centre"},
        {{24.9326, 67.1112}, "Asma M. Hashmi STEM Centre, NEDUET"},
        {{24.9332, 67.11083}, "Library Lawn"},
        {{24.93245, 67.1106}, "Dean Civil and Petroleum Engineering office"},
        {{24.9327, 67.1104}, "NED White House (Admin Block)"},
        {{24.93358, 67.10963}, "Transport section NED"},
        
        // --- YOUR ADDITION ---
        {{24.93095, 67.1139}, "Department of Mathematics"} // NEW ENTRY with precise coordinates
    };

    cout << "Processing GeoJSON features...\n";

    // Map to quickly check if a coordinate is one of the predefined (valid) locations
    map<pair<double, double>, int> predefined_location_ids;

    // First pass: Process LineStrings for road network
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

            // 1. Check for **EXACT** coordinate match first
            if (coord_to_id.count(key))
            {
                nodeId = coord_to_id[key];
                found_existing = true;
            }
            
            // 2. Check if a node already exists **VERY CLOSE** (e.g., <2 meters)
            if (!found_existing)
            {
                for (auto &existing : nodes)
                {
                    double dist = haversine(lat, lon, existing.second.lat, existing.second.lon);
                    if (dist < 2.0) // 2 meters threshold for road nodes
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

                // Check for connection to closest predefined location if within 100m (but don't rename yet)
                double min_dist = INF;
                string closest_name = "";
                for (const auto &loc : predefined_locations)
                {
                    double dist = haversine(lat, lon, loc.first.first, loc.first.second);
                    if (dist < min_dist && dist < 100)
                    {
                        min_dist = dist;
                        closest_name = loc.second;
                    }
                }
                // Only use the predefined name if it's extremely close, otherwise stick to generic for now
                if (min_dist < 5.0) 
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

    // Second pass: Add/Consolidate Predefined Locations (Prioritizing the valid names)
    for (const auto &loc : predefined_locations)
    {
        double lat = loc.first.first, lon = loc.first.second;
        string name = loc.second;
        pair<double, double> key = {lat, lon};

        int buildingId = -1;
        bool exists = false;
        
        // Find existing node that is VERY close (2m radius)
        for (auto &node_pair : nodes)
        {
            double d = haversine(lat, lon, node_pair.second.lat, node_pair.second.lon);
            if (d < 2.0) // If an existing road node is within 2 meters
            {
                // **Deduplication/Prioritization Logic:**
                // 1. Assign the VALID name to this existing node.
                node_pair.second.name = name; 
                buildingId = node_pair.first;
                exists = true;
                break;
            }
        }

        if (!exists)
        {
            // No close existing node found, create a new node for the location
            buildingId = id_counter++;
            nodes[buildingId] = {name, lat, lon};
            coord_to_id[key] = buildingId;
            if ((int)graph.size() <= buildingId)
                graph.resize(buildingId + 1);
        }
        
        // Register the ID of this canonical named node
        predefined_location_ids[key] = buildingId;

        // Connect to nearest road node (skip other named buildings/departments)
        double min_dist = INF;
        int nearest_node = -1;
        for (const auto &node_pair : nodes)
        {
            int nodeId = node_pair.first;
            const Node &node = node_pair.second;

            // Skip other named locations (to avoid connecting building-to-building directly)
            // unless the target is a generic Path_Node
            if (nodeId == buildingId || node.name.find("University") != string::npos || node.name.find("Department") != string::npos)
                continue; 

            double dist = haversine(lat, lon, node.lat, node.lon);
            if (dist < min_dist && dist < 300) 
            {
                min_dist = dist;
                nearest_node = nodeId;
            }
        }

        if (nearest_node != -1)
        {
            // Only add a new edge if the distance is greater than the 2m threshold, 
            // otherwise the consolidation already took care of it.
            if (min_dist > 2.0) {
                 graph[buildingId].push_back({nearest_node, min_dist, true});
                 graph[nearest_node].push_back({buildingId, min_dist, true});
            }
        }
    }

    // -----------------------------------------------------------------
    // 🌟 TARGETED DELETION OF UNWANTED NODES (455 to 467)
    // -----------------------------------------------------------------
    cout << "Starting targeted deletion of nodes 455-467...\n";
    
    const int DELETE_START = 455;
    const int DELETE_END = 467;

    // Step 1: Identify nodes to delete and potential remapping target (keeper)
    unordered_map<int, int> remap_table;
    vector<int> nodes_to_delete;

    for (int deleted_id = DELETE_START; deleted_id <= DELETE_END; ++deleted_id) {
        if (nodes.count(deleted_id)) {
            nodes_to_delete.push_back(deleted_id);
            int keeper_id = -1;

            if (deleted_id >= 0 && deleted_id < (int)graph.size()) {
                // Find the first non-deleted neighbor to become the keeper
                for (const auto& edge : graph[deleted_id]) {
                    // Check if the neighbor is outside the deletion range
                    if (edge.to != deleted_id && nodes.count(edge.to) && (edge.to < DELETE_START || edge.to > DELETE_END)) {
                        keeper_id = edge.to;
                        break; 
                    }
                }
            }
            
            if (keeper_id != -1) {
                remap_table[deleted_id] = keeper_id;
            }
        }
    }

    // Step 2: Update Graph Edges (Remapping Connections)
    vector<vector<Edge>> new_graph = graph; // Copy the graph for modification
    
    for (size_t u = 0; u < graph.size(); ++u) {
        // Only process nodes that are NOT being deleted
        if (remap_table.count(u)) continue; 

        vector<Edge> final_edges;
        map<int, double> connections; // Use map to track unique connections and min weight
        
        for (const auto& edge : graph[u]) {
            int v_original = edge.to;
            int v_new = remap_table.count(v_original) ? remap_table.at(v_original) : v_original;

            if ((int)u == v_new) continue; // Skip self-loops
            // Check if the remapped target is itself scheduled for deletion
            if (v_new >= DELETE_START && v_new <= DELETE_END && remap_table.count(v_original) == 0) continue; 
            if (v_new >= DELETE_START && v_new <= DELETE_END && remap_table.count(v_new)) continue;

            double final_weight = edge.weight;

            if (remap_table.count(v_original)) {
                // This edge was remapped. Recalculate distance to the keeper.
                if (nodes.count(v_new) && nodes.count(u)) {
                    final_weight = haversine(nodes[u].lat, nodes[u].lon, nodes[v_new].lat, nodes[v_new].lon);
                }
            }
            
            // Keep the minimum weight if multiple paths lead to the same node
            if (!connections.count(v_new) || final_weight < connections[v_new]) {
                connections[v_new] = final_weight;
            }
        }
        
        // Rebuild the edge list for node u
        for(const auto& conn : connections) {
            // Find the 'open' status from the original edge, defaulting to true
            bool edge_was_open = true; 
            for (const auto& original_edge : graph[u]) {
                if (original_edge.to == conn.first) {
                    edge_was_open = original_edge.open;
                    break;
                }
            }
            final_edges.push_back({conn.first, conn.second, edge_was_open});
        }
        
        new_graph[u] = final_edges;
    }

    // Step 3: Replace the graph with the new, corrected graph
    graph = new_graph;


    // Step 4: Final Deletion of Nodes from the map
    for (int deleted_id : nodes_to_delete) {
        nodes.erase(deleted_id);
    }

    cout << "Deletion complete. Remaining nodes: " << nodes.size() << ".\n";
    
    // Clean up empty vectors from graph size reduction (optional but good practice)
    int max_id_remaining = -1;
    for (const auto& pair : nodes) {
        if (pair.first > max_id_remaining) {
            max_id_remaining = pair.first;
        }
    }
    if (max_id_remaining != -1 && (int)graph.size() > max_id_remaining + 1) {
        graph.resize(max_id_remaining + 1);
    }


    cout << "Map loaded successfully: " << nodes.size() << " nodes, "
         << coord_to_id.size() << " unique coordinates" << endl;

    return true;
}

// ------------------- Dijkstra -------------------
vector<double> dijkstra(int n, const vector<vector<Edge>> &graph, int src, vector<int> &parent)
{
    vector<double> dist(n, INF);
    parent.assign(n, -1);
    if (src < 0 || src >= n)
        return dist;
    dist[src] = 0;
    using P = pair<double, int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    pq.push({0, src});
    while (!pq.empty())
    {
        auto [d, u] = pq.top();
        pq.pop();
        if (d != dist[u])
            continue;
        for (const auto &e : graph[u])
        {
            if (!e.open)
                continue;
            int v = e.to;
            double nd = d + e.weight;
            if (nd < dist[v])
            {
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }
    return dist;
}

// ------------------- Helpers -------------------
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

// ------------------- Find node by name -------------------
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

// ------------------- Format distance for display -------------------
string format_distance(double meters)
{
    if (meters < 1.0)
    {
        return to_string(int(meters * 100)) + " cm";
    }
    else if (meters < 1000.0)
    {
        return to_string(int(meters)) + "m";
    }
    else
    {
        return to_string(int(meters / 1000.0)) + "km " + to_string(int(fmod(meters, 1000.0))) + "m";
    }
}

// ------------------- Main -------------------
int main(int argc, char **argv)
{
    string filename = (argc > 1) ? argv[1] : "export.geojson";
    unordered_map<int, Node> nodes;
    vector<vector<Edge>> graph;

    if (!load_map(filename, nodes, graph))
    {
        cerr << "Failed to load map: " << filename << "\n";
        return 1;
    }

    cout << "Loaded NED University map from: " << filename << " (" << nodes.size() << " nodes)\n";

    while (true)
    {
        cout << "\n=== NED University Navigation System ===\n";
        cout << "1) Print all locations\n";
        cout << "2) Find shortest path between locations\n";
        cout << "3) Close a path (road closure)\n";
        cout << "4) Open a path\n";
        cout << "5) Reload map\n";
        cout << "6) Quit\n";
        cout << "Choice: ";
        cout.flush();

        int ch;
        cin >> ch;
        cin.ignore(numeric_limits<streamsize>::max(), '\n');

        if (ch == 1)
        {
            cout << "\n=== ALL NED UNIVERSITY LOCATIONS ===\n";
            for (auto &p : nodes)
            {
                cout << "ID: " << p.first
                     << " | Name: " << p.second.name
                     << " | Coordinates: (" << p.second.lat << ", " << p.second.lon << ")\n";
            }
        }
        else if (ch == 2)
        {
            cout << "\n=== Path Finding Options ===\n";
            cout << "1) Select from location list\n";
            cout << "2) Enter node IDs directly\n";
            cout << "Choice: ";

            int option;
            cin >> option;
            cin.ignore(numeric_limits<streamsize>::max(), '\n');

            int s, t;

            if (option == 1)
            {
                cout << "\nAvailable locations:\n";

                // Show unique location names with their IDs
                map<string, vector<int>> location_ids;
                for (auto &p : nodes)
                {
                    location_ids[p.second.name].push_back(p.first);
                }

                int counter = 1;
                map<int, string> menu_options; // number -> name
                for (auto &loc : location_ids)
                {
                    cout << counter << ") " << loc.first << " (IDs: ";
                    for (size_t i = 0; i < loc.second.size(); i++)
                    {
                        cout << loc.second[i];
                        if (i < loc.second.size() - 1)
                            cout << ", ";
                    }
                    cout << ")\n";
                    menu_options[counter] = loc.first;
                    counter++;
                }

                // Choose start
                cout << "\nChoose start location by number: ";
                int start_choice;
                if (!(cin >> start_choice))
                {
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    cout << "Invalid input\n";
                    continue;
                }

                // Choose destination
                cout << "Choose destination location by number: ";
                int dest_choice;
                if (!(cin >> dest_choice))
                {
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    cout << "Invalid input\n";
                    continue;
                }
                cin.ignore(numeric_limits<streamsize>::max(), '\n');

                if (menu_options.find(start_choice) == menu_options.end() ||
                    menu_options.find(dest_choice) == menu_options.end())
                {
                    cout << "Invalid choice. Please select numbers between 1 and " << menu_options.size() << "\n";
                    continue;
                }

                string start_name = menu_options[start_choice];
                string dest_name = menu_options[dest_choice];

                vector<int> start_nodes = find_nodes_by_name(nodes, start_name);
                vector<int> dest_nodes = find_nodes_by_name(nodes, dest_name);

                if (start_nodes.empty() || dest_nodes.empty())
                {
                    cout << "Could not find locations: " << start_name << " or " << dest_name << "\n";
                    continue;
                }

                // If multiple nodes exist, ask user to pick
                if (start_nodes.size() > 1)
                {
                    cout << "Multiple nodes found for " << start_name << ". Choose node ID: ";
                    for (int id : start_nodes)
                        cout << id << " ";
                    cout << "\n";
                    cin >> s;
                }
                else
                    s = start_nodes[0];

                if (dest_nodes.size() > 1)
                {
                    cout << "Multiple nodes found for " << dest_name << ". Choose node ID: ";
                    for (int id : dest_nodes)
                        cout << id << " ";
                    cout << "\n";
                    cin >> t;
                }
                else
                    t = dest_nodes[0];
            }
            else if (option == 2)
            {
                // Direct node ID input
                cout << "Enter start node ID: ";
                if (!(cin >> s))
                {
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    cout << "Invalid input\n";
                    continue;
                }
                cout << "Enter destination node ID: ";
                if (!(cin >> t))
                {
                    cin.clear();
                    cin.ignore(numeric_limits<streamsize>::max(), '\n');
                    cout << "Invalid input\n";
                    continue;
                }
                cin.ignore(numeric_limits<streamsize>::max(), '\n');

                if (nodes.find(s) == nodes.end() || nodes.find(t) == nodes.end())
                {
                    cout << "Invalid node ID. Please check the IDs and try again.\n";
                    continue;
                }
            }
            else
            {
                cout << "Invalid option\n";
                continue;
            }

            cout << "Finding path from " << nodes[s].name << " (ID: " << s
                 << ") to " << nodes[t].name << " (ID: " << t << ")\n";

            vector<int> parent;
            vector<double> dist = dijkstra((int)graph.size(), graph, s, parent);
            if (dist[t] >= INF)
            {
                cout << "No path found between " << nodes[s].name << " and " << nodes[t].name << "\n";

                // Debug: Check connectivity
                cout << "Debug: Start node has " << (s < (int)graph.size() ? graph[s].size() : 0) << " edges\n";
                cout << "Debug: Destination node has " << (t < (int)graph.size() ? graph[t].size() : 0) << " edges\n";

                // Check if nodes are isolated
                bool s_isolated = true;
                bool t_isolated = true;
                if (s < (int)graph.size()) {
                    for (const auto &edge : graph[s])
                    {
                        if (edge.open)
                            s_isolated = false;
                    }
                }
                if (t < (int)graph.size()) {
                    for (const auto &edge : graph[t])
                    {
                        if (edge.open)
                            t_isolated = false;
                    }
                }

                if (s_isolated)
                    cout << "Start node is isolated (no open edges)\n";
                if (t_isolated)
                    cout << "Destination node is isolated (no open edges)\n";

                continue;
            }

            vector<int> path;
            for (int cur = t; cur != -1; cur = parent[cur])
                path.push_back(cur);
            reverse(path.begin(), path.end());

            cout << "\n=== Route from " << nodes[s].name << " to " << nodes[t].name << " ===\n";
            cout << "Total distance: " << format_distance(dist[t]) << "\n";
            cout << "Path (" << path.size() << " nodes):\n";
            cout << "Start at: " << nodes[path[0]].name << "\n";

            // Calculate and display directions using left/right/straight
            double prevBearing = -1;
            for (size_t i = 0; i + 1 < path.size(); i++)
            {
                int u = path[i], v = path[i + 1];
                double w = find_edge_weight(graph, u, v);

                if (w > 0)
                {
                    double bearing = calculate_bearing(
                        nodes[u].lat, nodes[u].lon,
                        nodes[v].lat, nodes[v].lon);

                    string turnInstruction;
                    if (prevBearing != -1)
                    {
                        double turn = bearing - prevBearing;
                        if (turn < -180)
                            turn += 360;
                        if (turn > 180)
                            turn -= 360;

                        if (fabs(turn) < 30)
                            turnInstruction = "go straight";
                        else if (turn > 30)
                            turnInstruction = "turn right";
                        else
                            turnInstruction = "turn left";
                    }
                    else
                    {
                        turnInstruction = "start walking";
                    }

                    cout << "  -> " << turnInstruction << " for " << format_distance(w)
                          << " until " << nodes[v].name << "\n";

                    prevBearing = bearing;
                }
                else
                {
                    cout << "  -> Continue to " << nodes[v].name << "\n";
                }
            }
            cout << "Arrived at destination: " << nodes[t].name << "\n";
        }
        else if (ch == 3 || ch == 4)
        {
            int u, v;
            cout << "Enter path start ID and end ID: ";
            if (!(cin >> u >> v))
            {
                cin.clear();
                cin.ignore(numeric_limits<streamsize>::max(), '\n');
                cout << "Invalid input\n";
                continue;
            }
            cin.ignore(numeric_limits<streamsize>::max(), '\n');

            if (nodes.find(u) == nodes.end() || nodes.find(v) == nodes.end())
            {
                cout << "Invalid location ID\n";
                continue;
            }

            set_edge_open(graph, u, v, ch == 4);
            cout << "Path between " << nodes[u].name << " and " << nodes[v].name;
            cout << (ch == 4 ? " has been OPENED\n" : " has been CLOSED\n");
        }
        else if (ch == 5)
        {
            if (!load_map(filename, nodes, graph))
                cout << "Reload failed\n";
            else
                cout << "NED University map reloaded successfully\n";
        }
        else if (ch == 6)
        {
            cout << "Thank you for using NED University Navigation System!\n";
            break;
        }
        else
        {
            cout << "Invalid choice. Please enter 1-6.\n";
        }
    }
    return 0;
}