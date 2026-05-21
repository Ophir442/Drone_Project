#pragma once

#include "simulation.h"
#include <atomic>
#include <string>
#include <thread>

/**
 * @brief Minimal blocking HTTP/1.1 server exposing the Simulation over JSON.
 *
 * Single-listener, single-handler design: accept() and handle_client() run
 * sequentially on one server thread, so Simulation mutations driven by HTTP
 * requests are implicitly serialized. Adding a per-client worker would
 * race Simulation's internal state and must not be done without revisiting
 * the threading contract.
 *
 * Endpoints:
 *   GET    /api/state     — full state snapshot (bakeries, drones, customers)
 *   POST   /api/step      — advance one round, return new state
 *   POST   /api/customer  — add a customer
 *   DELETE /api/customer  — remove a customer by id
 *   POST   /api/init      — re-initialize from current config
 *   POST   /api/reset     — reset to the original config state
 *   GET    /api/config    — current configuration summary
 */
class HttpServer {
public:
	HttpServer(Simulation& sim, int port = 8080);
	~HttpServer();

	HttpServer(const HttpServer&)            = delete;
	HttpServer& operator=(const HttpServer&) = delete;

	/// Spawn the server thread and start listening.
	void start();

	/// Atomically tear down the listening socket and join the server thread.
	void stop();

private:
	Simulation&      sim;
	int              port;
	std::atomic<int> server_fd;   ///< -1 when no socket is open
	std::atomic<bool> running;
	std::thread      server_thread;

	void run_server();
	void handle_client(int client_fd);

	std::string handle_get_state();
	std::string handle_step();
	std::string handle_add_customer(const std::string& body);
	std::string handle_remove_customer(const std::string& body);
	std::string handle_initialize();
	std::string handle_reset();
	std::string handle_config();

	std::string make_response(int status, const std::string& body);
	std::string make_cors_headers();
	std::string parse_body(const std::string& request);
	std::string parse_path(const std::string& request);
	std::string parse_method(const std::string& request);
};
