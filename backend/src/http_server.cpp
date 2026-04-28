#include "http_server.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <sstream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

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
	if (server_fd >= 0) {
		shutdown(server_fd, SHUT_RDWR);
		close(server_fd);
		server_fd = -1;
	}
	if (server_thread.joinable()) {
		server_thread.join();
	}
}

void HttpServer::run_server() {
	server_fd = socket(AF_INET, SOCK_STREAM, 0);
	if (server_fd < 0) {
		std::cerr << "Failed to create socket" << std::endl;
		return;
	}

	int opt = 1;
	setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

	struct sockaddr_in addr;
	memset(&addr, 0, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(port);

	if (bind(server_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
		std::cerr << "Failed to bind to port " << port << std::endl;
		close(server_fd);
		server_fd = -1;
		return;
	}

	if (listen(server_fd, 10) < 0) {
		std::cerr << "Failed to listen" << std::endl;
		close(server_fd);
		server_fd = -1;
		return;
	}

	std::cout << "HTTP server listening on port " << port << std::endl;

	while (running) {
		struct sockaddr_in client_addr;
		socklen_t client_len = sizeof(client_addr);
		int client_fd = accept(server_fd, (struct sockaddr*)&client_addr, &client_len);
		if (client_fd < 0) {
			if (running) {
				std::cerr << "Accept failed" << std::endl;
			}
			continue;
		}
		handle_client(client_fd);
	}
}

void HttpServer::handle_client(int client_fd) {
	char buffer[8192];
	memset(buffer, 0, sizeof(buffer));
	ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
	if (bytes <= 0) {
		close(client_fd);
		return;
	}

	std::string request(buffer, bytes);
	std::string method = parse_method(request);
	std::string path = parse_path(request);
	std::string body = parse_body(request);

	std::string response;

	// Handle CORS preflight
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
	state["base"] = {{"x", sim.get_base_pos().x}, {"y", sim.get_base_pos().y}};
	state["total_delivered"] = sim.get_total_delivered();
	state["total_customers_served"] = sim.get_total_customers_served();

	// Bakeries
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

	// Customers
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

	// Drones
	json jdrones = json::array();
	for (const auto& d : sim.get_drones()) {
		json route = json::array();
		for (const auto& node : d.planned_route) {
			json jnode;
			jnode["pos"] = {{"x", node.pos.x}, {"y", node.pos.y}};
			jnode["entity_id"] = node.entity_id;
			jnode["bread_amount"] = node.bread_amount;
			jnode["committed"] = node.committed;
			switch (node.type) {
				case RouteNodeType::BAKERY_PICKUP: jnode["type"] = "pickup"; break;
				case RouteNodeType::CUSTOMER_DELIVERY: jnode["type"] = "delivery"; break;
				case RouteNodeType::BASE_RETURN: jnode["type"] = "base_return"; break;
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

	// Last resolved intents
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
	bool can_continue = sim.step_round();
	json result;
	result["continued"] = can_continue;
	result["round"] = sim.get_round();
	// Include full state in response
	json state = json::parse(handle_get_state());
	result.merge_patch(state);
	return result.dump();
}

std::string HttpServer::handle_add_customer(const std::string& body) {
	try {
		json j = json::parse(body);
		double x = j.value("x", 50.0);
		double y = j.value("y", 50.0);
		int qty = j.value("order_quantity", 3);
		std::string name = j.value("name", "");
		sim.add_customer(x, y, qty, name);
		return json({{"success", true}, {"message", "Customer added"}}).dump();
	} catch (const std::exception& e) {
		return json({{"success", false}, {"error", e.what()}}).dump();
	}
}

std::string HttpServer::handle_remove_customer(const std::string& body) {
	try {
		json j = json::parse(body);
		int id = j.value("id", -1);
		if (id < 0) {
			return json({{"success", false}, {"error", "Invalid customer ID"}}).dump();
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
	j["grid_width"] = cfg.grid_width;
	j["grid_height"] = cfg.grid_height;
	j["drone_templates"] = static_cast<int>(cfg.drone_configs.size());
	j["num_active_drones"] = static_cast<int>(sim.get_drones().size());
	j["num_bakeries"] = static_cast<int>(cfg.bakery_configs.size());
	j["num_customers"] = static_cast<int>(cfg.customer_configs.size());
	j["max_rounds"] = cfg.max_rounds;
	j["grasp_iterations"] = cfg.grasp_iterations;
	j["rcl_size"] = cfg.rcl_size;
	j["thread_count"] = cfg.thread_count;
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
		case 200: status_text = "OK"; break;
		case 404: status_text = "Not Found"; break;
		default: status_text = "Unknown"; break;
	}

	std::ostringstream ss;
	ss << "HTTP/1.1 " << status << " " << status_text << "\r\n";
	ss << make_cors_headers();
	ss << "Content-Type: application/json\r\n";
	ss << "Content-Length: " << body.size() << "\r\n";
	ss << "Connection: close\r\n";
	ss << "\r\n";
	ss << body;
	return ss.str();
}

std::string HttpServer::parse_method(const std::string& request) {
	auto space = request.find(' ');
	if (space == std::string::npos) return "";
	return request.substr(0, space);
}

std::string HttpServer::parse_path(const std::string& request) {
	auto first_space = request.find(' ');
	if (first_space == std::string::npos) return "";
	auto second_space = request.find(' ', first_space + 1);
	if (second_space == std::string::npos) return "";
	std::string path = request.substr(first_space + 1, second_space - first_space - 1);
	// Remove query string
	auto q = path.find('?');
	if (q != std::string::npos) path = path.substr(0, q);
	return path;
}

std::string HttpServer::parse_body(const std::string& request) {
	auto pos = request.find("\r\n\r\n");
	if (pos == std::string::npos) return "";
	return request.substr(pos + 4);
}
