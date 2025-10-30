#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <string>
#include <limits>
#include <algorithm>
#include <unordered_map>
using namespace std;

struct Node {
    string name;
    int x = 0;
    int y = 0;
};
struct Edge {
    int to;
    int weight;
    bool open;
};

const int INF = 1e9;

static inline string trim(const string &s) {
    size_t a = s.find_first_not_of(" \t\r\n");
    if (a == string::npos) return "";
    size_t b = s.find_last_not_of(" \t\r\n");
    return s.substr(a, b - a + 1);
}

bool load_map(const string &filename,
              unordered_map<int, Node> &nodes,
              vector<vector<Edge>> &graph)
{
    ifstream in(filename);
    if (!in) return false;

    nodes.clear();
    graph.clear();

    string line;
    bool reading_edges = false;
    int maxNodeId = -1;

    while (getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;

        if (line == "---") {
            reading_edges = true;
            continue;
        }

        stringstream ss(line);
        if (!reading_edges) {
            int id;
            if (!(ss >> id)) continue;
            string name;
            ss >> ws;
            if (ss.peek() == '"') {
                ss.get();
                getline(ss, name, '"');
                ss >> ws;
            } else {
                ss >> name;
            }
            int x = 0, y = 0;
            if (!(ss >> x >> y)) {
                x = y = 0;
            }
            nodes[id] = {name, x, y};
            maxNodeId = max(maxNodeId, id);
        } else {
            int u, v, w;
            int openFlag = 1;
            stringstream ess(line);
            if (!(ess >> u >> v >> w)) continue;
            if (!(ess >> openFlag)) openFlag = 1;
            int need = max(u, v);
            if ((int)graph.size() <= need) graph.resize(need + 1);
            graph[u].push_back({v, w, static_cast<bool>(openFlag)});
            graph[v].push_back({u, w, static_cast<bool>(openFlag)});
        }
    }

    if ((int)graph.size() <= maxNodeId)
        graph.resize(maxNodeId + 1);

    return true;
}

vector<int> dijkstra(int n, const vector<vector<Edge>> &graph, int src, vector<int> &parent)
{
    vector<int> dist(n, INF);
    parent.assign(n, -1);
    if (src < 0 || src >= n) return dist;
    dist[src] = 0;
    using P = pair<int,int>;
    priority_queue<P, vector<P>, greater<P>> pq;
    pq.push({0, src});
    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d != dist[u]) continue;
        for (const auto &e : graph[u]) {
            if (!e.open) continue;
            int v = e.to;
            int nd = d + e.weight;
            if (nd < dist[v]) {
                dist[v] = nd;
                parent[v] = u;
                pq.push({nd, v});
            }
        }
    }
    return dist;
}

void print_nodes(const unordered_map<int, Node> &nodes) {
    cout << "\navailable nodes (id : name [x,y])\n";
    vector<pair<int, Node>> v;
    v.reserve(nodes.size());
    for (auto &p : nodes) v.push_back(p);
    sort(v.begin(), v.end(), [](auto &a, auto &b){ return a.first < b.first; });
    for (auto &p : v) {
        cout << p.first << " : " << p.second.name
             << " (" << p.second.x << "," << p.second.y << ")\n";
    }
    cout << '\n';
}

void set_edge_open(vector<vector<Edge>> &graph, int u, int v, bool open) {
    if (u < 0 || v < 0 || u >= (int)graph.size() || v >= (int)graph.size()) return;
    for (auto &e : graph[u]) if (e.to == v) e.open = open;
    for (auto &e : graph[v]) if (e.to == u) e.open = open;
}

int find_edge_weight(const vector<vector<Edge>> &graph, int u, int v) {
    if (u < 0 || v < 0 || u >= (int)graph.size() || v >= (int)graph.size()) return -1;
    for (const auto &e : graph[u]) if (e.to == v) return e.weight;
    return -1;
}

string direction(const Node &prev, const Node &cur, const Node &next) {
    int dx1 = cur.x - prev.x;
    int dy1 = cur.y - prev.y;
    int dx2 = next.x - cur.x;
    int dy2 = next.y - cur.y;
    int cross = dx1 * dy2 - dy1 * dx2;
    if (cross == 0) return "walk straight";
    else if (cross > 0) return "turn left";
    else return "turn right";
}

bool read_int_from_prompt(const string &prompt, int &out) {
    string line;
    cout << prompt << flush;
    if (!getline(cin, line)) return false;
    stringstream ss(line);
    if (!(ss >> out)) return false;
    return true;
}

// ✅ New helper for waiting
void wait_for_enter() {
    string dummy;
    cout << "press Enter to continue..." << flush;
    getline(cin, dummy);
}

int main(int argc, char** argv) {
    string filename = (argc > 1) ? argv[1] : "campus_map.txt";
    unordered_map<int, Node> nodes;
    vector<vector<Edge>> graph;

    if (!load_map(filename, nodes, graph)) {
        cerr << "failed to load map. create campus_map.txt or pass path as argument\n";
        return 1;
    }

    cout << "loaded map from: " << filename << " (" << nodes.size() << " nodes)\n";

    string line;
    while (true) {
        cout << "\nmenu:\n";
        cout << " 1) print nodes\n";
        cout << " 2) find shortest path\n";
        cout << " 3) close an edge (mark path closed)\n";
        cout << " 4) open an edge\n";
        cout << " 5) reload map from file\n";
        cout << " 6) quit\n";

        int ch;
        if (!read_int_from_prompt("choice: ", ch)) {
            cout << "\nexiting...\n";
            break;
        }

        if (ch == 1) {
            print_nodes(nodes);
        }
        else if (ch == 2) {
            print_nodes(nodes);
            int s, t;
            if (!read_int_from_prompt("enter start id: ", s)) { cout << "invalid start id\n"; continue; }
            if (!read_int_from_prompt("enter destination id: ", t)) { cout << "invalid destination id\n"; continue; }

            if (s < 0 || s >= (int)graph.size() || t < 0 || t >= (int)graph.size()) {
                cout << "invalid node ids\n";
                continue;
            }

            vector<int> parent;
            vector<int> dist = dijkstra((int)graph.size(), graph, s, parent);
            if (dist[t] >= INF) {
                cout << "no path found from id " << s << " to id " << t << "\n";
                continue;
            }

            vector<int> path;
            for (int cur = t; cur != -1; cur = parent[cur]) path.push_back(cur);
            reverse(path.begin(), path.end());

            cout << "\nfound path (total " << dist[t] << " m):\n";

            for (size_t i = 0; i + 1 < path.size(); ++i) {
                int u = path[i];
                int v = path[i+1];
                int w = find_edge_weight(graph, u, v);
                if (w < 0) w = 0;

                if (i > 0) {
                    int prev = path[i-1];
                    string dir = direction(nodes[prev], nodes[u], nodes[v]);
                    cout << dir << " and walk " << w << " m to " << nodes[v].name << "\n";
                } else {
                    cout << "walk " << w << " m to " << nodes[v].name << "\n";
                }

                wait_for_enter();
            }

            cout << "arrived at " << nodes[t].name << " (total " << dist[t] << " m)\n";
        }
        else if (ch == 3 || ch == 4) {
            print_nodes(nodes);
            int u, v;
            if (!read_int_from_prompt("enter u id: ", u)) { cout << "invalid u id\n"; continue; }
            if (!read_int_from_prompt("enter v id: ", v)) { cout << "invalid v id\n"; continue; }

            if (u < 0 || v < 0 || u >= (int)graph.size() || v >= (int)graph.size()) {
                cout << "invalid node ids\n";
                continue;
            }
            bool open = (ch == 4);
            set_edge_open(graph, u, v, open);
            cout << (open ? "edge opened\n" : "edge closed\n");
        }
        else if (ch == 5) {
            if (!load_map(filename, nodes, graph)) {
                cout << "reload failed\n";
            } else {
                cout << "map reloaded\n";
            }
        }
        else if (ch == 6) {
            cout << "bye\n";
            break;
        } else {
            cout << "invalid choice\n";
        }
    }

    return 0;
}
