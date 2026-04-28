#include "simulation.hpp"
#include "http_server.hpp"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <thread>
#include <csignal>

static HttpServer* global_server = nullptr;

void signal_handler(int) {
	if (global_server) {
		global_server->stop();
	}
	std::cout << "\nServer stopped." << std::endl;
	exit(0);
}

SimConfig load_config(const std::string& path) {
	std::ifstream file(path);
	if (!file.is_open()) {
		std::cerr << "ERROR: Config file not found at " << path << "\n";
		exit(1);
	}

	nlohmann::json j;
	file >> j;

	SimConfig config;

	// World: physical layout shared by all entities.
	const auto& world = j.at("world");
	config.grid_width  = world.value("grid_width", 100);
	config.grid_height = world.value("grid_height", 100);
	config.base_pos.x  = world.at("base_pos").value("x", 50.0);
	config.base_pos.y  = world.at("base_pos").value("y", 50.0);

	// Simulation: high-level loop parameters.
	const auto& sim = j.at("simulation");
	config.max_rounds         = sim.value("max_rounds", 50);
	config.priority_increment = sim.value("priority_increment", 1.0);

	// Algorithm: GRASP / parallelism tuning.
	const auto& algo = j.at("algorithm");
	config.grasp_iterations = algo.value("grasp_iterations", 5);
	config.rcl_size         = algo.value("rcl_size", 3);
	int hw = static_cast<int>(std::thread::hardware_concurrency());
	config.thread_count     = algo.value("thread_count", hw > 0 ? hw : 4);

	// Predefined drone fleet (each entry is one drone, fixed for the run).
	for (const auto& jd : j.at("drones")) {
		DroneConfig dc;
		dc.velocity = jd.value("velocity", 5.0);
		dc.capacity = jd.value("capacity", 20);
		config.drone_configs.push_back(dc);
	}

	// Bakeries (positions, capacity, starting stock, production distribution).
	for (const auto& jb : j.at("bakeries")) {
		BakeryConfig bc;
		bc.pos.x             = jb.at("pos").value("x", 0.0);
		bc.pos.y             = jb.at("pos").value("y", 0.0);
		bc.capacity          = jb.value("capacity", 50);
		bc.initial_inventory = jb.value("initial_inventory", 25);
		for (const auto& entry : jb.at("production_distribution")) {
			bc.production_distribution.push_back({entry[0].get<int>(), entry[1].get<double>()});
		}
		config.bakery_configs.push_back(bc);
	}

	// Initial customers. Priority is NOT here — it starts at 1.0 and grows
	// at runtime per the spec's w_t+1 = w_t + 1 rule for unserved orders.
	for (const auto& jc : j.at("customers")) {
		CustomerConfig cc;
		cc.pos.x          = jc.at("pos").value("x", 0.0);
		cc.pos.y          = jc.at("pos").value("y", 0.0);
		cc.order_quantity = jc.value("order_quantity", 3);
		config.customer_configs.push_back(cc);
	}

	return config;
}

int main(int argc, char* argv[]) {
	std::string config_path = "config.json";
	bool server_mode = false;
	int port = 8080;

	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if (arg == "--server" || arg == "-s") {
			server_mode = true;
		} else if (arg == "--port" || arg == "-p") {
			if (i + 1 < argc) {
				port = std::stoi(argv[++i]);
			}
		} else {
			config_path = arg;
		}
	}

	SimConfig config = load_config(config_path);

	if (server_mode) {
		// Interactive server mode
		Simulation sim(config);
		sim.initialize();

		HttpServer server(sim, port);
		global_server = &server;
		std::signal(SIGINT, signal_handler);
		std::signal(SIGTERM, signal_handler);

		std::cout << "Starting in server mode on port " << port << std::endl;
		std::cout << "Press Ctrl+C to stop." << std::endl;
		server.start();

		// Keep main thread alive
		while (true) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	} else {
		// Batch mode (original behavior)
		Simulation sim(config);
		sim.run();
	}

	return 0;
}
