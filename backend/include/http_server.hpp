#pragma once

#include "simulation.hpp"
#include <string>
#include <functional>
#include <thread>
#include <atomic>

class HttpServer {
public:
	HttpServer(Simulation& sim, int port = 8080);
	~HttpServer();

	void start();
	void stop();

private:
	Simulation& sim;
	int port;
	int server_fd;
	std::atomic<bool> running;
	std::thread server_thread;

	void run_server();
	void handle_client(int client_fd);

	// Route handlers
	std::string handle_get_state();
	std::string handle_step();
	std::string handle_add_customer(const std::string& body);
	std::string handle_remove_customer(const std::string& body);
	std::string handle_initialize();
	std::string handle_reset();
	std::string handle_config();

	// HTTP helpers
	std::string make_response(int status, const std::string& body);
	std::string make_cors_headers();
	std::string parse_body(const std::string& request);
	std::string parse_path(const std::string& request);
	std::string parse_method(const std::string& request);
};
