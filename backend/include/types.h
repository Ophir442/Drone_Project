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
	std::vector<std::pair<int, double>> production_distribution;
};

struct Customer {
	int id;
	Position pos;
	int order_quantity;
	double priority_weight;

	bool operator<(const Customer& other) const {
		return priority_weight < other.priority_weight;
	}
};

enum class RouteNodeType {
	BAKERY_PICKUP,
	CUSTOMER_DELIVERY,
	BASE_RETURN
};

struct RouteNode {
	RouteNodeType type;
	Position pos;
	int entity_id;
	int customer_id = -1;
	int bread_amount;
	bool committed;
};

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
	int route_progress;
	bool is_idle;
	std::vector<DeliveryEvent> pending_deliveries;
};

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
