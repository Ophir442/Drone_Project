#pragma once

#include "distance.h"
#include "grasp.h"
#include "thread_pool.h"
#include "types.h"
#include <memory>
#include <random>
#include <unordered_set>
#include <vector>

/**
 * @brief Orchestrates one parallel bread-delivery simulation.
 *
 * Per-round pipeline:
 *   1. STATE UPDATE  — bakery production + drone motion (parallel)
 *   2. GRASP PLAN    — N iterations across the thread pool (parallel)
 *   3. CONTENTION    — bakery-side resolution by score (serial)
 *   4. COMMIT        — stamp routes, reposition idle drones (serial)
 *
 * Stages 3+4 together form a two-phase commit:
 *   - Phase 1 (planning): GRASP workers run on snapshots and produce
 *     non-binding @c Intent records. Because every worker operates on a
 *     private copy of the bakery inventory, multiple workers may "promise"
 *     the same bread to different customers.
 *   - Phase 2 (commit): @c resolve_bakery_contention reconciles those
 *     promises against the real, shared inventory — by score, highest first
 *     — and emits the actual allocation. Only allocations that survive
 *     contention are written to drone routes.
 *
 * Customers live in a flat vector; in-place bookkeeping (priority bumps,
 * delivery debits) is O(n) instead of the O(n log n) drain-and-rebuild that
 * a std::priority_queue forces. GRASP sorts a one-shot snapshot per round
 * for its priority-order pass.
 *
 * All mutation of the customer vector and the assigned-id sets happens on
 * the main thread.
 */
class Simulation {
public:
	explicit Simulation(const SimConfig& config);

	/// Validate the config, initialize state, then loop step_round() until
	/// every customer is served and every drone is idle (or until the
	/// no-progress halt clause fires).
	void run();

	/// Build initial bakeries, drones, and customer queue. Idempotent if
	/// reset() is called first.
	void initialize();

	/// Tear down all per-run state and re-initialize from the original config.
	void reset();

	/// Advance the simulation by exactly one round. Returns false when the
	/// simulation can terminate (all served, or no progress possible).
	bool step_round();

	// ---- customer mutation (server / live-edit) ----
	void add_customer(double x, double y, int order_quantity,
	                  const std::string& name = "");
	void remove_customer(int customer_id);

	// ---- snapshots (read-only views for the HTTP layer) ----
	const std::vector<Bakery>&   get_bakeries() const { return bakeries; }
	const std::vector<Drone>&    get_drones()   const { return drones; }
	const std::vector<Customer>& get_all_customers() const { return customer_queue; }
	int get_round() const { return current_round; }
	const SimConfig& get_config() const { return config; }
	const Position&  get_base_pos() const { return config.base_pos; }
	const std::vector<Intent>& get_last_intents() const { return last_resolved_intents; }
	int get_total_delivered()        const { return total_bread_delivered; }
	int get_total_customers_served() const { return total_customers_served; }

private:
	SimConfig config;

	std::vector<Bakery>            bakeries;
	std::vector<Customer>          customer_queue;   ///< flat, unsorted; GRASP sorts per round
	std::vector<Drone>             drones;
	DeliveryGraph                  delivery_graph;

	std::unique_ptr<ThreadPool> thread_pool;
	std::mt19937                rng;

	int next_customer_id;
	int next_drone_id;
	int current_round;
	int total_bread_delivered;
	int total_customers_served;

	std::vector<Intent>      last_resolved_intents;
	std::unordered_set<int>  assigned_customer_ids;
	std::unordered_set<int>  served_this_round;

	// ---- validation ----
	void validate_config() const;

	// ---- pipeline ----
	void          stage1_state_update();
	GraspSolution stage2_3_assignment();
	void          stage4_commit(GraspSolution& solution);

	// ---- per-drone motion ----
	void advance_drone(Drone& drone);
	void apply_delivery_events();

	// ---- fleet management (auto-scaling) ----
	void  maybe_spawn_drone();
	bool  should_spawn_drone() const;
	Drone spawn_drone();
	Drone* find_drone(int id);

	// ---- idle repositioning ----
	void          reposition_idle_drones(const std::unordered_set<int>& assigned_drone_ids);
	const Bakery* choose_repositioning_target(const Drone& drone) const;
};
