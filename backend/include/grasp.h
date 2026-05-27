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
	/// Aggregate post-2-Opt score: total delivered "value" divided by total
	/// fleet route time. Captures 2-Opt improvements that the per-intent
	/// original_score (frozen at candidate-selection time) cannot.
	double                                final_score = 0.0;
};

/**
 * @brief Greedy Randomized Adaptive Search Procedure for drone↔customer assignment.
 *
 * Each iteration builds one feasible solution by sampling from a Restricted
 * Candidate List (RCL) and then improves the resulting per-drone routes with
 * 2-Opt. Multiple iterations explore different neighborhoods of the
 * assignment space; the best-scoring solution wins.
 *
 * The solver is thread-safe: @c drones and @c bakeries are read-only and may
 * be shared across threads, each iteration mutates only its own local scratch
 * buffers, and each thread supplies its own @c std::mt19937. The
 * @c DistanceCache passed to the constructor is likewise shared read-only.
 */
class GraspSolver {
public:
	/**
	 * @brief Construct a solver bound to a static graph and a per-round cache.
	 *
	 * @param graph      static bakery+base distance graph (read-only).
	 * @param rcl_size   maximum size of the Restricted Candidate List per customer.
	 * @param iterations total iterations to run across all threads (informational).
	 * @param cache      per-round distance cache; must outlive the solver.
	 *
	 * Storing the cache as a const reference (rather than a mutable pointer set
	 * by run_single_iteration) makes the solver a pure functor: no method
	 * mutates state across calls, so the same solver is safe to reuse across
	 * iterations without re-construction.
	 */
	GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations,
	            const DistanceCache& cache);

	/**
	 * @brief Build one feasible solution and improve it with 2-Opt.
	 *
	 * Data-oriented design: @p drones and @p bakeries are read-only const
	 * references shared across all worker threads — only their immutable
	 * fields (positions, capacities, velocities, ids, existing committed
	 * routes) are read. All per-iteration mutation lives in three lightweight
	 * primitive scratch buffers allocated inside the call (bakery inventory,
	 * per-drone evolving routes, per-drone cargo). No Drone or Bakery struct
	 * is ever deep-copied, so the parallel allocator is never contended on the
	 * inner vectors/strings those structs carry.
	 *
	 * @param customers active (unassigned) customer set, max-priority first.
	 * @param drones    shared read-only fleet (positions/capacities/routes).
	 * @param bakeries  shared read-only bakery set (positions/capacities).
	 * @param base_pos  depot position used for return-leg time estimation.
	 * @param rng       thread-local Mersenne Twister.
	 */
	GraspSolution run_single_iteration(
		const std::vector<Customer>& customers,
		const std::vector<Drone>& drones,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos,
		std::mt19937& rng);

private:
	/// A single (drone, bakery, amount) option for one customer.
	struct Candidate {
		int    drone_idx;
		int    bakery_id;
		int    amount;
		double score;
	};

	const DeliveryGraph& graph;
	int                  rcl_size;
	int                  iterations;
	const DistanceCache& cache;

	// ---- distance helpers ----
	double distance_between(const Position& a, int a_node,
	                        const Position& b, int b_node) const;
	int node_id_for(const RouteNode& node) const;
	int drone_start_node(const Drone& drone) const;
	int base_node_id() const;

	// ---- route timing ----
	// Routes are passed in explicitly (the drone's *local* evolving route),
	// not read from Drone::planned_route, so timing reflects the in-progress
	// scratch plan rather than the shared read-only fleet state.
	double path_distance(const Position& start_pos, int start_node,
	                     const std::vector<RouteNode>& route, std::size_t start_idx,
	                     const Position* end_pos = nullptr, int end_node = -1) const;
	double compute_route_time(const Drone& drone,
	                          const std::vector<RouteNode>& route,
	                          const Position& base_pos) const;
	double compute_route_time_with_assignment(
		const Drone& drone, const std::vector<RouteNode>& route,
		const Bakery& bakery, const Customer& customer, int amount,
		const Position& base_pos) const;

	// ---- selection ----
	// Bakery stock is read from @p inventory (the local scratch buffer, indexed
	// by bakery id), never from Bakery::current_inventory — the Bakery objects
	// are shared read-only and carry only positions/capacities here.
	BakeryAssignment find_best_bakery(
		const Drone& drone, const std::vector<RouteNode>& route,
		const Customer& customer, const std::vector<Bakery>& bakeries,
		const std::vector<int>& inventory, const Position& base_pos) const;

	std::vector<Candidate> build_candidates(
		const Customer& customer,
		const std::vector<Drone>& drones,
		const std::vector<std::vector<RouteNode>>& local_routes,
		const std::vector<int>& local_drone_loads,
		const std::vector<Bakery>& bakeries,
		const std::vector<int>& local_inventory,
		const Position& base_pos) const;

	// ---- 2-opt ----
	void apply_two_opt(std::vector<RouteNode>& route,
	                   const Position& start_pos, int start_node) const;
	bool is_route_valid(const std::vector<RouteNode>& route) const;
	bool is_route_valid_after_reverse(
		const std::vector<RouteNode>& route,
		std::size_t i, std::size_t j) const;
};
