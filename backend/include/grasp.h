#pragma once

#include "distance.h"
#include "types.h"
#include <map>
#include <random>
#include <vector>

/// Result of GraspSolver::find_best_bakery: which bakery, and how much it
/// can supply for the customer this iteration.
struct BakeryAssignment {
	int bakery_id;
	int achievable_amount;
};

/// One iteration's complete output: a flat list of intents plus the
/// per-drone planned routes those intents implied.
struct GraspSolution {
	std::vector<Intent>                   intents;
	std::map<int, std::vector<RouteNode>> drone_routes;
};

/**
 * @brief Greedy Randomized Adaptive Search Procedure for drone↔customer assignment.
 *
 * Each iteration builds one feasible solution by sampling from a Restricted
 * Candidate List (RCL) and then improves the resulting per-drone routes with
 * 2-Opt. Multiple iterations explore different neighborhoods of the
 * assignment space; the best-scoring solution wins.
 *
 * The solver is thread-safe given per-thread snapshots of @c drones and
 * @c bakeries plus a per-thread @c std::mt19937. The @c DistanceCache is
 * shared read-only across threads.
 */
class GraspSolver {
public:
	/**
	 * @brief Construct a solver bound to a static delivery graph.
	 * @param graph      static bakery+base distance graph (read-only).
	 * @param rcl_size   maximum size of the Restricted Candidate List per customer.
	 * @param iterations total iterations to run across all threads (informational).
	 */
	GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations);

	/**
	 * @brief Build one feasible solution and improve it with 2-Opt.
	 *
	 * Operates on private snapshots of @p drones and @p bakeries — the
	 * caller is responsible for cloning state per worker thread.
	 *
	 * @param customers active (unassigned) customer set, max-priority first.
	 * @param drones    per-thread snapshot of the fleet.
	 * @param bakeries  per-thread snapshot of inventory levels.
	 * @param base_pos  depot position used for return-leg time estimation.
	 * @param cache     shared per-round distance cache (read-only).
	 * @param rng       thread-local Mersenne Twister.
	 */
	GraspSolution run_single_iteration(
		const std::vector<Customer>& customers,
		const std::vector<Drone>& drones,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos,
		const DistanceCache& cache,
		std::mt19937& rng);

	/// Sum of per-intent original_scores; higher = better assignment.
	static double evaluate_solution(const std::vector<Intent>& intents);

private:
	/// A single (drone, bakery, amount) option for one customer.
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
