#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <string>

struct Position {
	double x;
	double y;

	double distance_to(const Position& other) const {
		double dx = x - other.x;
		double dy = y - other.y;
		return std::sqrt(dx * dx + dy * dy);
	}

	bool operator==(const Position& other) const {
		return x == other.x && y == other.y;
	}
};

struct Bakery {
	int id;
	Position pos;
	int current_inventory;
	int capacity;
	// Each pair: (amount, probability) for production each round
	std::vector<std::pair<int, double>> production_distribution;
};

// Comparator for max-heap based on priority_weight
struct Customer {
	int id;
	Position pos;
	int order_quantity;
	double priority_weight;

	// For max-heap: lower priority_weight = lower priority in queue
	bool operator<(const Customer& other) const {
		return priority_weight < other.priority_weight;
	}
};

// Route node: either a bakery pickup or customer delivery
enum class RouteNodeType {
	BAKERY_PICKUP,
	CUSTOMER_DELIVERY,
	BASE_RETURN
};

struct RouteNode {
	RouteNodeType type;
	Position pos;
	int entity_id;        // bakery_id or customer_id (legacy)
	int customer_id = -1; // customer this pickup/delivery serves; -1 for repositioning/base
	int bread_amount;     // amount to pick up or deliver
	bool committed;       // true if bread already collected from bakery in prior round
};

// Deferred delivery event — recorded during parallel drone advancement,
// applied sequentially to avoid races on shared state.
struct DeliveryEvent {
	int customer_id;
	int bread_delivered;
};

struct Drone {
	int id;
	Position current_pos;
	double velocity;
	int current_load;
	int max_capacity;
	std::vector<RouteNode> planned_route;
	int route_progress;   // index into planned_route for current target
	bool is_idle;
	std::vector<DeliveryEvent> pending_deliveries; // filled during parallel advance, drained sequentially
};

struct Intent {
	int drone_id;
	int bakery_id;
	int requested_bread_amount;
	int customer_id;
	double original_score;
};

// Drone template (user-defined) — system spawns new drones from these as needed
struct DroneConfig {
	double velocity;
	int capacity;
};

// Per-bakery configuration (user-defined)
struct BakeryConfig {
	Position pos;
	int capacity;
	int initial_inventory;
	std::vector<std::pair<int, double>> production_distribution;
};

// Per-customer configuration (user-defined)
struct CustomerConfig {
	Position pos;
	int order_quantity;
};

// Configuration for the simulation
struct SimConfig {
	int grid_width;
	int grid_height;
	int max_rounds;
	double priority_increment;
	int grasp_iterations;
	int rcl_size;            // top-k for RCL
	int thread_count;
	Position base_pos;

	// User-defined entities
	std::vector<DroneConfig> drone_configs;
	std::vector<BakeryConfig> bakery_configs;
	std::vector<CustomerConfig> customer_configs;
};
