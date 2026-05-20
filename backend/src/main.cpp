#include "simulation.h"
#include "http_server.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>

using namespace std;
using nlohmann::json;

static HttpServer* global_server = nullptr;

void signal_handler(int) {
	if (global_server) global_server->stop();
	cout << "\nServer stopped." << endl;
	exit(0);
}

SimConfig load_config(const string& path) {
	ifstream file(path);
	if (!file.is_open()) {
		cerr << "ERROR: Config file not found at " << path << "\n";
		exit(1);
	}

	json j;
	file >> j;

	SimConfig config;

	const auto& world = j.at("world");
	config.grid_width  = world.value("grid_width", 100);
	config.grid_height = world.value("grid_height", 100);
	config.base_pos.x  = world.at("base_pos").value("x", 50.0);
	config.base_pos.y  = world.at("base_pos").value("y", 50.0);

	const auto& sim = j.at("simulation");
	config.priority_increment = sim.value("priority_increment", 1.0);

	const auto& algo = j.at("algorithm");
	config.grasp_iterations = algo.value("grasp_iterations", 5);
	config.rcl_size         = algo.value("rcl_size", 3);
	int hw = static_cast<int>(thread::hardware_concurrency());
	config.thread_count     = algo.value("thread_count", hw > 0 ? hw : 4);

	const auto& dt = j.at("drone_template");
	config.drone_template.capacity_min  = dt.value("capacity_min", 10);
	config.drone_template.capacity_max  = dt.value("capacity_max", 20);
	config.drone_template.velocity_min  = dt.value("velocity_min", 3.0);
	config.drone_template.velocity_max  = dt.value("velocity_max", 8.0);
	config.drone_template.initial_count = dt.value("initial_count", 1);

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

	// Customer priority isn't loaded — it starts at 1.0 and grows at runtime
	// per the spec rule for unserved orders.
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
	string config_path = "config.json";
	bool server_mode = false;
	int port = 8080;

	for (int i = 1; i < argc; ++i) {
		string arg = argv[i];
		if (arg == "--server" || arg == "-s") {
			server_mode = true;
		} else if (arg == "--port" || arg == "-p") {
			if (i + 1 < argc) port = stoi(argv[++i]);
		} else {
			config_path = arg;
		}
	}

	SimConfig config = load_config(config_path);

	if (server_mode) {
		Simulation sim(config);
		sim.initialize();

		HttpServer server(sim, port);
		global_server = &server;
		signal(SIGINT, signal_handler);
		signal(SIGTERM, signal_handler);

		cout << "Starting in server mode on port " << port << endl;
		cout << "Press Ctrl+C to stop." << endl;
		server.start();

		while (true) this_thread::sleep_for(chrono::seconds(1));
	} else {
		Simulation sim(config);
		sim.run();
	}

	return 0;
}
