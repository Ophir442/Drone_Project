#pragma once

#include <vector>
#include <utility>
#include <cmath>
#include <string>

/// 2D point on the simulation grid.
struct Position {
	double x;
	double y;

	/// Euclidean distance between this point and `other`.
	double distance_to(const Position& other) const noexcept {
		double dx = x - other.x;
		double dy = y - other.y;
		return std::sqrt(dx * dx + dy * dy);
	}

	bool operator==(const Position& other) const noexcept {
		return x == other.x && y == other.y;
	}
};

/// Bread producer with bounded capacity and stochastic per-round production.
struct Bakery {
	int id;
	Position pos;
	int current_inventory;
	int capacity;
	/// (amount, probability) pairs; probabilities should sum to 1.0.
	std::vector<std::pair<int, double>> production_distribution;
};

/// Pending order. priority_weight grows each round the customer is unserved.
struct Customer {
	int id;
	Position pos;
	int order_quantity;
	double priority_weight;

	/// Max-heap ordering: higher priority leaves the queue first.
	bool operator<(const Customer& other) const noexcept {
		return priority_weight < other.priority_weight;
	}
};

/// Action carried out at a route waypoint.
enum class RouteNodeType {
	BAKERY_PICKUP,
	CUSTOMER_DELIVERY,
	BASE_RETURN
};

/// One step of a drone's plan: where to go and what to do on arrival.
struct RouteNode {
	RouteNodeType type;
	Position pos;
	int entity_id;            ///< bakery_id or customer_id depending on type
	int customer_id = -1;     ///< the customer this node serves (-1 for repositioning)
	int bread_amount;
	bool committed;           ///< true once the drone has executed this node
};

/// A delivery that landed during a round; applied after parallel motion.
struct DeliveryEvent {
	int customer_id;
	int bread_delivered;
};

/// Carrier with a planned route and a per-instance velocity.
struct Drone {
	int id;
	Position current_pos;
	double velocity;
	int current_load;
	int max_capacity;
	std::vector<RouteNode> planned_route;
	int route_progress;
	bool is_idle;
	std::vector<DeliveryEvent> pending_deliveries;
};

/// GRASP output: a drone wants `requested_bread_amount` from a bakery for a customer.
struct Intent {
	int drone_id;
	int bakery_id;
	int requested_bread_amount;
	int customer_id;
	double original_score;
};

struct DroneTemplate {
	int capacity;
	double velocity_min;
	double velocity_max;
	int initial_count;
};

struct BakeryConfig {
	Position pos;
	int capacity;
	int initial_inventory;
	std::vector<std::pair<int, double>> production_distribution;
};

struct CustomerConfig {
	Position pos;
	int order_quantity;
};

/// Loaded from config.json; immutable for the duration of a run.
struct SimConfig {
	int grid_width;
	int grid_height;
	int max_rounds;
	double priority_increment;
	int grasp_iterations;
	int rcl_size;
	int thread_count;
	Position base_pos;

	DroneTemplate drone_template;
	std::vector<BakeryConfig> bakery_configs;
	std::vector<CustomerConfig> customer_configs;
};
