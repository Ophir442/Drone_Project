/**
 * @file main.cpp
 * @brief Entry point: parse CLI, load config.json, run sim or HTTP server.
 *
 * The shutdown path is intentionally signal-safe: the SIGINT/SIGTERM
 * handler does nothing but flip an atomic flag. The main thread polls the
 * flag and performs the actual teardown (joins, iostream, exit) outside the
 * signal context, where iostream and joins are legal.
 */

#include "http_server.h"
#include "simulation.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

using nlohmann::json;

namespace {

/// Set by the signal handler; polled by main() to trigger graceful shutdown.
std::atomic<bool> g_shutdown_requested{false};

/// Async-signal-safe: writes one atomic. No iostream, no joins, no exit().
extern "C" void signal_handler(int /*sig*/) {
	g_shutdown_requested.store(true, std::memory_order_relaxed);
}

/// Parse config.json into a fully-populated SimConfig. Throws on parse
/// errors; semantic validation is deferred to Simulation::validate_config.
SimConfig load_config(const std::string& path) {
	std::ifstream file(path);
	if (!file.is_open()) {
		std::cerr << "ERROR: Config file not found at " << path << "\n";
		std::exit(1);
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
	const int hw = static_cast<int>(std::thread::hardware_concurrency());
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
			bc.production_distribution.push_back(
				{entry[0].get<int>(), entry[1].get<double>()});
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

/// Parse CLI args: positional path, --server/-s, --port/-p N.
struct CliArgs {
	std::string config_path = "config.json";
	bool        server_mode = false;
	int         port        = 8080;
};

CliArgs parse_cli(int argc, char* argv[]) {
	CliArgs args;
	for (int i = 1; i < argc; ++i) {
		const std::string arg = argv[i];
		if (arg == "--server" || arg == "-s") {
			args.server_mode = true;
		} else if (arg == "--port" || arg == "-p") {
			if (i + 1 < argc) args.port = std::stoi(argv[++i]);
		} else {
			args.config_path = arg;
		}
	}
	return args;
}

}  // namespace


int main(int argc, char* argv[]) {
	const CliArgs args = parse_cli(argc, argv);

	SimConfig config = load_config(args.config_path);

	if (args.server_mode) {
		Simulation sim(config);
		sim.initialize();

		HttpServer server(sim, args.port);
		std::signal(SIGINT,  signal_handler);
		std::signal(SIGTERM, signal_handler);

		server.start();

		while (!g_shutdown_requested.load(std::memory_order_relaxed)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(200));
		}

		server.stop();
		return 0;
	}

	Simulation sim(config);
	sim.run();
	return 0;
}
