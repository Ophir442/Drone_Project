#pragma once

#include "types.h"
#include "distance.h"
#include <vector>
#include <random>
#include <map>

/// Result of `find_best_bakery`: which bakery and how much it can supply.
struct BakeryAssignment {
	int bakery_id;
	int achievable_amount;
};

/// Output of a single GRASP iteration.
struct GraspSolution {
	std::vector<Intent> intents;
	std::map<int, std::vector<RouteNode>> drone_routes;
};

/// Greedy-randomized adaptive search for drone↔customer assignment.
/// Each iteration builds one solution by sampling from a restricted
/// candidate list (RCL), then improves the resulting routes with 2-Opt.
class GraspSolver {
public:
	GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations);

	/// Run one GRASP iteration on a private snapshot of state.
	/// Thread-safe given private copies of `drones` / `bakeries` and a
	/// thread-local `rng`. `cache` is shared (read-only) across threads.
	GraspSolution run_single_iteration(
		const std::vector<Customer>& customers,
		const std::vector<Drone>& drones,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos,
		const DistanceCache& cache,
		std::mt19937& rng);

	/// Sum of per-intent scores; higher = better assignment.
	static double evaluate_solution(const std::vector<Intent>& intents);

private:
	/// A single (drone, bakery, amount) option considered for one customer.
	struct Candidate {
		int    drone_idx;
		int    bakery_id;
		int    amount;
		double score;
	};

	const DeliveryGraph& graph;
	int rcl_size;
	int iterations;
	const DistanceCache* cache = nullptr;

	// ---- distance helpers ----
	double distance_between(const Position& a, int a_node,
	                        const Position& b, int b_node) const;
	int node_id_for(const RouteNode& node) const;
	int drone_start_node(const Drone& drone) const;
	int base_node_id() const;

	// ---- route timing ----
	double path_distance(const Position& start_pos, int start_node,
	                     const std::vector<RouteNode>& route, std::size_t start_idx,
	                     const Position* end_pos = nullptr, int end_node = -1) const;
	double compute_route_time(const Drone& drone, const Position& base_pos) const;
	double compute_route_time_with_assignment(
		const Drone& drone, const Bakery& bakery,
		const Customer& customer, int amount,
		const Position& base_pos) const;

	// ---- selection ----
	BakeryAssignment find_best_bakery(
		const Drone& drone, const Customer& customer,
		const std::vector<Bakery>& bakeries, const Position& base_pos) const;

	std::vector<Candidate> build_candidates(
		const Customer& customer,
		const std::vector<Drone>& drones,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos) const;

	// ---- 2-opt ----
	void apply_two_opt(std::vector<RouteNode>& route,
	                   const Position& start_pos, int start_node) const;
	bool is_route_valid(const std::vector<RouteNode>& route) const;
};
