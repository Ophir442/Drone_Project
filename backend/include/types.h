#pragma once

#include <cmath>
#include <string>
#include <utility>
#include <vector>

/**
 * @brief 2D point on the simulation grid.
 *
 * All entity positions and the depot ("base") are expressed as Positions.
 * Distances are pure Euclidean — the simulation does not model obstacles.
 */
struct Position {
	double x;
	double y;

	/// Euclidean distance from this point to @p other.
	double distance_to(const Position& other) const noexcept {
		const double dx = x - other.x;
		const double dy = y - other.y;
		return std::sqrt(dx * dx + dy * dy);
	}
};

/**
 * @brief Bread producer with bounded inventory and stochastic per-round output.
 *
 * Each round a single bucket from @c production_distribution is sampled; the
 * sampled @c amount is added to @c current_inventory but clamped at @c capacity
 * (overproduction is wasted, modelling a silo overflow).
 */
struct Bakery {
	int id;
	Position pos;
	int current_inventory;
	int capacity;
	/// (amount, probability) buckets. Probabilities should sum to 1.0.
	std::vector<std::pair<int, double>> production_distribution;
};

/**
 * @brief Pending order. @c priority_weight grows each round the order is unserved
 * so that long-waiting customers float to the top of the heap.
 */
struct Customer {
	int id;
	Position pos;
	int order_quantity;
	double priority_weight;

	/// Max-heap ordering: higher @c priority_weight leaves the queue first.
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

/**
 * @brief One step of a drone's plan: where to go and what to do on arrival.
 *
 * @c committed flips to true the moment the drone executes this node. The
 * commit flag protects already-executed nodes from two threats:
 *   - apply_allocation_to_route mutating an already-delivered amount
 *   - 2-Opt swapping a committed waypoint back into the mutable suffix.
 */
struct RouteNode {
	RouteNodeType type;
	Position pos;
	int entity_id;            ///< bakery_id or customer_id, depending on @c type
	int customer_id = -1;     ///< customer this node serves (-1 for repositioning)
	int bread_amount;
	bool committed;
};

/// A delivery that landed during a round; drained into the customer queue
/// on the main thread after parallel motion completes.
struct DeliveryEvent {
	int customer_id;
	int bread_delivered;
};

/**
 * @brief Carrier with a planned route and a per-instance velocity.
 *
 * Velocity and max_capacity are drawn from the DroneTemplate distribution at
 * spawn time and never mutate afterwards.
 */
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

/// GRASP output: a drone wants @c requested_bread_amount from a bakery for a customer.
struct Intent {
	int drone_id;
	int bakery_id;
	int requested_bread_amount;
	int customer_id;
	double original_score;
};

/// Spawn distribution for newly-created drones. Each new drone draws its
/// velocity from U[velocity_min, velocity_max] and capacity from
/// U{capacity_min, ..., capacity_max}.
struct DroneTemplate {
	int capacity_min;
	int capacity_max;
	double velocity_min;
	double velocity_max;
	int initial_count;
};

/// Per-bakery static configuration loaded from JSON.
struct BakeryConfig {
	Position pos;
	int capacity;
	int initial_inventory;
	std::vector<std::pair<int, double>> production_distribution;
};

/// Per-customer static configuration loaded from JSON.
struct CustomerConfig {
	Position pos;
	int order_quantity;
};

/// Top-level simulation configuration; loaded from config.json once and
/// then immutable for the duration of a run.
struct SimConfig {
	int grid_width;
	int grid_height;
	double priority_increment;
	int grasp_iterations;
	int rcl_size;
	int thread_count;
	Position base_pos;

	DroneTemplate drone_template;
	std::vector<BakeryConfig> bakery_configs;
	std::vector<CustomerConfig> customer_configs;
};
