/**
 * @file http_server.cpp
 * @brief Blocking single-threaded HTTP/1.1 façade over the Simulation.
 *
 * See http_server.h for the endpoint surface. handle_client() runs on the
 * server thread synchronously — there is intentionally no per-client
 * worker, so Simulation mutations are serialized by the accept loop.
 */

#include "http_server.h"

#include <arpa/inet.h>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

using json = nlohmann::json;

HttpServer::HttpServer(Simulation& sim, int port)
	: sim(sim), port(port), server_fd(-1), running(false) {}

HttpServer::~HttpServer() {
	stop();
}

void HttpServer::start() {
	running = true;
	server_thread = std::thread(&HttpServer::run_server, this);
}

void HttpServer::stop() {
	running = false;
	const int fd = server_fd.exchange(-1);
	if (fd >= 0) {
		shutdown(fd, SHUT_RDWR);
		close(fd);
	}
	if (server_thread.joinable()) server_thread.join();
}

void HttpServer::run_server() {
	const int fd = socket(AF_INET, SOCK_STREAM, 0);
	if (fd < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		return;
	}
	server_fd.store(fd);

	int opt = 1;
	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	sockaddr_in addr{};
	addr.sin_family      = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port        = htons(port);

	if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
		std::cerr << "Failed to bind to port " << port << std::endl;
		close(fd);
		server_fd.store(-1);
		return;
	}

	if (listen(fd, 10) < 0) {
		std::cerr << "Failed to listen" << std::endl;
		close(fd);
		server_fd.store(-1);
		return;
	}

	std::cout << "HTTP server listening on port " << port << std::endl;

	// INVARIANT: handle_client must be called synchronously here.
	// Spawning a per-client worker would race Simulation's internal state.
	while (running.load()) {
		sockaddr_in client_addr{};
		socklen_t   client_len = sizeof(client_addr);
		const int client_fd = accept(fd,
			reinterpret_cast<sockaddr*>(&client_addr), &client_len);
		if (client_fd < 0) {
			if (running.load()) std::cerr << "Accept failed" << std::endl;
			continue;
		}
		handle_client(client_fd);
	}
}

void HttpServer::handle_client(int client_fd) {
	char buffer[8192];
	std::memset(buffer, 0, sizeof(buffer));
	const ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
	if (bytes <= 0) {
		close(client_fd);
		return;
	}

	const std::string request(buffer, bytes);
	const std::string method = parse_method(request);
	const std::string path   = parse_path(request);
	const std::string body   = parse_body(request);

	std::string response;
	if (method == "OPTIONS") {
		response = make_response(200, "");
	} else if (path == "/api/state" && method == "GET") {
		response = make_response(200, handle_get_state());
	} else if (path == "/api/step" && method == "POST") {
		response = make_response(200, handle_step());
	} else if (path == "/api/customer" && method == "POST") {
		response = make_response(200, handle_add_customer(body));
	} else if (path == "/api/customer" && method == "DELETE") {
		response = make_response(200, handle_remove_customer(body));
	} else if (path == "/api/init" && method == "POST") {
		response = make_response(200, handle_initialize());
	} else if (path == "/api/reset" && method == "POST") {
		response = make_response(200, handle_reset());
	} else if (path == "/api/config" && method == "GET") {
		response = make_response(200, handle_config());
	} else {
		response = make_response(404, "{\"error\": \"Not found\"}");
	}

	send(client_fd, response.c_str(), response.size(), 0);
	close(client_fd);
}

std::string HttpServer::handle_get_state() {
	json state;
	state["round"] = sim.get_round();
	state["base"]  = {{"x", sim.get_base_pos().x}, {"y", sim.get_base_pos().y}};
	state["total_delivered"]        = sim.get_total_delivered();
	state["total_customers_served"] = sim.get_total_customers_served();

	json jbakeries = json::array();
	for (const auto& b : sim.get_bakeries()) {
		jbakeries.push_back({
			{"id", b.id},
			{"pos", {{"x", b.pos.x}, {"y", b.pos.y}}},
			{"current_inventory", b.current_inventory},
			{"capacity", b.capacity}
		});
	}
	state["bakeries"] = jbakeries;

	json jcustomers = json::array();
	for (const auto& c : sim.get_all_customers()) {
		jcustomers.push_back({
			{"id", c.id},
			{"pos", {{"x", c.pos.x}, {"y", c.pos.y}}},
			{"order_quantity", c.order_quantity},
			{"priority_weight", c.priority_weight}
		});
	}
	state["customers"] = jcustomers;

	json jdrones = json::array();
	for (const auto& d : sim.get_drones()) {
		json route = json::array();
		for (const auto& node : d.planned_route) {
			json jnode;
			jnode["pos"]          = {{"x", node.pos.x}, {"y", node.pos.y}};
			jnode["entity_id"]    = node.entity_id;
			jnode["bread_amount"] = node.bread_amount;
			jnode["committed"]    = node.committed;
			switch (node.type) {
				case RouteNodeType::BAKERY_PICKUP:     jnode["type"] = "pickup";      break;
				case RouteNodeType::CUSTOMER_DELIVERY: jnode["type"] = "delivery";    break;
				case RouteNodeType::BASE_RETURN:       jnode["type"] = "base_return"; break;
			}
			route.push_back(jnode);
		}

		jdrones.push_back({
			{"id", d.id},
			{"pos", {{"x", d.current_pos.x}, {"y", d.current_pos.y}}},
			{"velocity", d.velocity},
			{"current_load", d.current_load},
			{"max_capacity", d.max_capacity},
			{"is_idle", d.is_idle},
			{"route_progress", d.route_progress},
			{"planned_route", route}
		});
	}
	state["drones"] = jdrones;

	json jintents = json::array();
	for (const auto& intent : sim.get_last_intents()) {
		jintents.push_back({
			{"drone_id", intent.drone_id},
			{"bakery_id", intent.bakery_id},
			{"requested_bread_amount", intent.requested_bread_amount},
			{"customer_id", intent.customer_id},
			{"original_score", intent.original_score}
		});
	}
	state["resolved_intents"] = jintents;

	return state.dump();
}

std::string HttpServer::handle_step() {
	const bool can_continue = sim.step_round();
	json result;
	result["continued"] = can_continue;
	result["round"]     = sim.get_round();
	result.merge_patch(json::parse(handle_get_state()));
	return result.dump();
}

std::string HttpServer::handle_add_customer(const std::string& body) {
	try {
		json j = json::parse(body);
		const double x      = j.value("x", 50.0);
		const double y      = j.value("y", 50.0);
		const int    qty    = j.value("order_quantity", 3);
		const std::string name = j.value("name", "");
		if (qty <= 0) {
			return json({{"success", false},
			             {"error", "order_quantity must be > 0"}}).dump();
		}
		sim.add_customer(x, y, qty, name);
		return json({{"success", true}, {"message", "Customer added"}}).dump();
	} catch (const std::exception& e) {
		return json({{"success", false}, {"error", e.what()}}).dump();
	}
}

std::string HttpServer::handle_remove_customer(const std::string& body) {
	try {
		json j = json::parse(body);
		const int id = j.value("id", -1);
		if (id < 0) {
			return json({{"success", false},
			             {"error", "Invalid customer ID"}}).dump();
		}
		sim.remove_customer(id);
		return json({{"success", true}, {"message", "Customer removed"}}).dump();
	} catch (const std::exception& e) {
		return json({{"success", false}, {"error", e.what()}}).dump();
	}
}

std::string HttpServer::handle_initialize() {
	sim.initialize();
	return handle_get_state();
}

std::string HttpServer::handle_reset() {
	sim.reset();
	return handle_get_state();
}

std::string HttpServer::handle_config() {
	const auto& cfg = sim.get_config();
	json j;
	j["grid_width"]         = cfg.grid_width;
	j["grid_height"]        = cfg.grid_height;
	j["num_active_drones"]  = static_cast<int>(sim.get_drones().size());
	j["drone_capacity_min"] = cfg.drone_template.capacity_min;
	j["drone_capacity_max"] = cfg.drone_template.capacity_max;
	j["drone_velocity_min"] = cfg.drone_template.velocity_min;
	j["drone_velocity_max"] = cfg.drone_template.velocity_max;
	j["num_bakeries"]       = static_cast<int>(cfg.bakery_configs.size());
	j["num_customers"]      = static_cast<int>(cfg.customer_configs.size());
	j["grasp_iterations"]   = cfg.grasp_iterations;
	j["rcl_size"]           = cfg.rcl_size;
	j["thread_count"]       = cfg.thread_count;
	return j.dump();
}

std::string HttpServer::make_cors_headers() {
	return "Access-Control-Allow-Origin: *\r\n"
	       "Access-Control-Allow-Methods: GET, POST, DELETE, OPTIONS\r\n"
	       "Access-Control-Allow-Headers: Content-Type\r\n";
}

std::string HttpServer::make_response(int status, const std::string& body) {
	std::string status_text;
	switch (status) {
		case 200: status_text = "OK";        break;
		case 404: status_text = "Not Found"; break;
		default:  status_text = "Unknown";   break;
	}

	std::ostringstream ss;
	ss << "HTTP/1.1 " << status << " " << status_text << "\r\n"
	   << make_cors_headers()
	   << "Content-Type: application/json\r\n"
	   << "Content-Length: " << body.size() << "\r\n"
	   << "Connection: close\r\n\r\n"
	   << body;
	return ss.str();
}

std::string HttpServer::parse_method(const std::string& request) {
	const auto space = request.find(' ');
	if (space == std::string::npos) return "";
	return request.substr(0, space);
}

std::string HttpServer::parse_path(const std::string& request) {
	const auto first = request.find(' ');
	if (first == std::string::npos) return "";
	const auto second = request.find(' ', first + 1);
	if (second == std::string::npos) return "";
	std::string path = request.substr(first + 1, second - first - 1);
	const auto q = path.find('?');
	if (q != std::string::npos) path = path.substr(0, q);
	return path;
}

std::string HttpServer::parse_body(const std::string& request) {
	const auto pos = request.find("\r\n\r\n");
	if (pos == std::string::npos) return "";
	return request.substr(pos + 4);
}
