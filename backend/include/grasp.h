#pragma once

#include "types.hpp"
#include "distance.hpp"
#include <vector>
#include <random>
#include <map>

// Result of bakery selection: which bakery, and how much can actually be drawn.
struct BakeryAssignment {
	int bakery_id;          // -1 if no usable bakery
	int achievable_amount;  // <= customer.order_quantity; 0 if none usable
};

// Output of one full GRASP construction + 2-Opt local search iteration.
// Carries both the proposed intents AND the per-drone 2-Opt-optimized routes
// so the commit phase can preserve the ordering rather than rebuild naively.
struct GraspSolution {
	std::vector<Intent> intents;
	std::map<int, std::vector<RouteNode>> drone_routes; // drone_id -> full optimized planned_route
};

class GraspSolver {
public:
	GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations);

	// Run a single GRASP construction + 2-Opt local search iteration.
	// Returns the proposed intents AND the per-drone optimized routes.
	GraspSolution run_single_iteration(
		const std::vector<Customer>& customers,
		const std::vector<Drone>& drones,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos,
		std::mt19937& rng
	);

	// Evaluate total solution quality (sum of scores across all intents)
	static double evaluate_solution(const std::vector<Intent>& intents);

private:
	const DeliveryGraph& graph;
	int rcl_size;
	int iterations;

	// Compute total route completion time for a drone
	double compute_route_time(const Drone& drone, const Position& base_pos) const;

	// Compute route time with a hypothetical new assignment appended
	double compute_route_time_with_assignment(
		const Drone& drone,
		const Bakery& bakery,
		const Customer& customer,
		const Position& base_pos
	) const;

	// Find best bakery for a customer; returns the bakery and the actually-
	// achievable amount (truncated to bakery inventory in the fallback case).
	BakeryAssignment find_best_bakery(
		const Drone& drone,
		const Customer& customer,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos
	) const;

	// Apply 2-Opt local search on a drone's planned route
	// Respects precedence constraints: pickup before delivery
	// start_node: graph node ID of the starting position (-1 if dynamic)
	void apply_two_opt(std::vector<RouteNode>& route, const Position& start_pos, int start_node = -1) const;

	// Check if route respects precedence (pickup before delivery)
	bool is_route_valid(const std::vector<RouteNode>& route) const;

	// Compute total distance of a route
	double route_distance(const std::vector<RouteNode>& route, const Position& start_pos, int start_node) const;

	// Map a RouteNode to its static graph node ID (-1 if dynamic)
	int node_id_for(const RouteNode& node) const;
};
