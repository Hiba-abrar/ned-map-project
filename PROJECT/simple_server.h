#pragma once
#include <iostream>
#include <string>
#include <map>
#include <functional>
#include <thread>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

class SimpleServer {
public:
    struct Request {
        std::string method;
        std::string path;
        std::string body;
        std::map<std::string, std::string> headers;
    };
    
    struct Response {
        int status = 200;
        std::string body;
        std::map<std::string, std::string> headers;
        
        void set_content(const std::string& content, const std::string& content_type) {
            body = content;
            headers["Content-Type"] = content_type;
        }
    };
    
    using Handler = std::function<void(const Request&, Response&)>;
    
private:
    std::map<std::pair<std::string, std::string>, Handler> handlers;
    int server_socket;
    bool running = false;
    
public:
    SimpleServer() {
#ifdef _WIN32
        WSADATA wsaData;
        WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif
    }
    
    ~SimpleServer() {
#ifdef _WIN32
        closesocket(server_socket);
        WSACleanup();
#else
        close(server_socket);
#endif
    }
    
    void Get(const std::string& path, Handler handler) {
        handlers[{"GET", path}] = handler;
    }
    
    void Post(const std::string& path, Handler handler) {
        handlers[{"POST", path}] = handler;
    }
    
    void Options(const std::string& path, Handler handler) {
        handlers[{"OPTIONS", path}] = handler;
    }
    
    bool listen(const std::string& host, int port) {
        server_socket = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket < 0) return false;
        
        int opt = 1;
#ifdef _WIN32
        setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt));
#else
        setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#endif
        
        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);
        
        if (bind(server_socket, (sockaddr*)&address, sizeof(address)) < 0) return false;
        if (::listen(server_socket, 3) < 0) return false;
        
        running = true;
        
        while (running) {
            sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            int client_socket = accept(server_socket, (sockaddr*)&client_addr, &client_len);
            
            if (client_socket >= 0) {
                std::thread([this, client_socket]() {
                    handle_client(client_socket);
                }).detach();
            }
        }
        
        return true;
    }
    
private:
    void handle_client(int client_socket) {
        char buffer[4096] = {0};
        recv(client_socket, buffer, sizeof(buffer), 0);
        
        Request req = parse_request(std::string(buffer));
        Response res;
        
        // Add CORS headers
        res.headers["Access-Control-Allow-Origin"] = "*";
        res.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS";
        res.headers["Access-Control-Allow-Headers"] = "Content-Type";
        
        // Find handler
        auto it = handlers.find({req.method, req.path});
        if (it != handlers.end()) {
            it->second(req, res);
        } else {
            // Try pattern matching for OPTIONS
            if (req.method == "OPTIONS") {
                auto opt_it = handlers.find({"OPTIONS", ".*"});
                if (opt_it != handlers.end()) {
                    opt_it->second(req, res);
                }
            } else {
                res.status = 404;
                res.body = "Not Found";
            }
        }
        
        std::string response = build_response(res);
        send(client_socket, response.c_str(), response.length(), 0);
        
#ifdef _WIN32
        closesocket(client_socket);
#else
        close(client_socket);
#endif
    }
    
    Request parse_request(const std::string& raw) {
        Request req;
        std::istringstream stream(raw);
        std::string line;
        
        // Parse request line
        if (std::getline(stream, line)) {
            std::istringstream line_stream(line);
            line_stream >> req.method >> req.path;
        }
        
        // Parse headers
        while (std::getline(stream, line) && line != "\r") {
            size_t colon = line.find(':');
            if (colon != std::string::npos) {
                std::string key = line.substr(0, colon);
                std::string value = line.substr(colon + 2);
                if (!value.empty() && value.back() == '\r') {
                    value.pop_back();
                }
                req.headers[key] = value;
            }
        }
        
        // Parse body
        std::string body_line;
        while (std::getline(stream, body_line)) {
            req.body += body_line;
        }
        
        return req;
    }
    
    std::string build_response(const Response& res) {
        std::ostringstream response;
        response << "HTTP/1.1 " << res.status << " OK\r\n";
        
        for (const auto& header : res.headers) {
            response << header.first << ": " << header.second << "\r\n";
        }
        
        response << "Content-Length: " << res.body.length() << "\r\n";
        response << "\r\n";
        response << res.body;
        
        return response.str();
    }
};