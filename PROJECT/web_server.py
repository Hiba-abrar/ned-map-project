import http.server
import socketserver
import json
import math
from urllib.parse import parse_qs

locations = [
    {"id": 0, "name": "NED University Main Gate", "lat": 24.9299167, "lon": 67.1156389},
    {"id": 1, "name": "NED University Library", "lat": 24.9335294, "lon": 67.1110976},
    {"id": 2, "name": "NED University Admin Block", "lat": 24.9328525, "lon": 67.1099341},
    {"id": 3, "name": "CSIT Labs", "lat": 24.9313470, "lon": 67.1139814},
    {"id": 4, "name": "Main Auditorium", "lat": 24.9320192, "lon": 67.1125931},
    {"id": 5, "name": "DMS Cafeteria", "lat": 24.9326224, "lon": 67.1144837}
]

def distance(lat1, lon1, lat2, lon2):
    dx = (lat1 - lat2) * 111000
    dy = (lon1 - lon2) * 111000
    return math.sqrt(dx*dx + dy*dy)

class Handler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/api/locations':
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(locations).encode())
        elif self.path == '/':
            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
            with open('index.html', 'r') as f:
                self.wfile.write(f.read().encode())
        elif self.path == '/style.css':
            self.send_response(200)
            self.send_header('Content-type', 'text/css')
            self.end_headers()
            with open('style.css', 'r') as f:
                self.wfile.write(f.read().encode())
        elif self.path == '/script.js':
            self.send_response(200)
            self.send_header('Content-type', 'text/javascript')
            self.end_headers()
            with open('script.js', 'r') as f:
                self.wfile.write(f.read().encode())
        else:
            self.send_error(404)
    
    def do_POST(self):
        if self.path == '/api/route':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode('utf-8'))
            
            start_name = data['start']
            end_name = data['end']
            
            start_loc = next((l for l in locations if l['name'] == start_name), None)
            end_loc = next((l for l in locations if l['name'] == end_name), None)
            
            if start_loc and end_loc:
                dist = distance(start_loc['lat'], start_loc['lon'], end_loc['lat'], end_loc['lon'])
                result = {
                    "success": True,
                    "path": [start_loc, end_loc],
                    "steps": [{"instruction": f"Go to {end_name}", "distance": int(dist)}],
                    "total_distance": int(dist)
                }
            else:
                result = {"success": False, "error": "Location not found"}
            
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode())
    
    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        self.end_headers()

PORT = 8080
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print(f"Server running at http://localhost:{PORT}")
    httpd.serve_forever()