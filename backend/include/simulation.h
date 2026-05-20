#pragma once

#include "types.h"
#include "distance.h"
#include "grasp.h"
#include "thread_pool.h"
#include <vector>
#include <queue>
#include <random>
#include <set>
#include <memory>

/// Orchestrates one parallel bread-delivery run.
///
/// Each round follows a 4-stage pipeline:
///   1. state update    — bakery production + drone motion (parallel)
///   2. GRASP planning  — N iterations across the thread pool (parallel)
///   3. contention      — bakery-side resolution by score (serial)
///   4. commit          — write routes, reposition idle drones (serial)
///
/// All mutation of the customer priority_queue and assigned-id sets happens
/// on the main thread so the priority_queue stays single-writer.
class Simulation {
public:
	explicit Simulation(const SimConfig& config);

	/// Validate the config, initialize state, and loop step_round() until
	/// every customer is served and every drone is idle.
	void run();

	void initialize();
	void reset();
	bool step_round();

	// ---- customer mutation (server / live-edit) ----
	void add_customer(double x, double y, int order_quantity,
	                  const std::string& name = "");
	void remove_customer(int customer_id);

	// ---- snapshots ----
	const std::vector<Bakery>& get_bakeries() const { return bakeries; }
	const std::vector<Drone>&  get_drones()   const { return drones; }
	std::vector<Customer>      get_all_customers() const;
	int get_round() const { return current_round; }
	const SimConfig& get_config() const { return config; }
	const Position&  get_base_pos() const { return config.base_pos; }
	const std::vector<Intent>& get_last_intents() const { return last_resolved_intents; }
	int get_total_delivered()        const { return total_bread_delivered; }
	int get_total_customers_served() const { return total_customers_served; }

private:
	SimConfig config;

	std::vector<Bakery>            bakeries;
	std::priority_queue<Customer>  customer_queue;
	std::vector<Drone>             drones;
	DeliveryGraph                  delivery_graph;

	std::unique_ptr<ThreadPool> thread_pool;
	std::mt19937                rng;

	int next_customer_id;
	int next_drone_id;
	int current_round;
	int total_bread_delivered;
	int total_customers_served;

	std::vector<Intent> last_resolved_intents;
	std::set<int>       assigned_customer_ids;
	std::set<int>       served_this_round;

	// ---- validation ----
	void validate_config() const;

	// ---- pipeline ----
	void          stage1_state_update();
	GraspSolution stage2_3_assignment();
	void          stage4_commit(GraspSolution& solution);

	// ---- per-drone motion ----
	void advance_drone(Drone& drone);
	void apply_delivery_events();

	// ---- fleet management ----
	void  maybe_spawn_drone();
	bool  should_spawn_drone() const;
	Drone spawn_drone();
	Drone* find_drone(int id);

	// ---- idle repositioning ----
	void          reposition_idle_drones(const std::set<int>& assigned_drone_ids);
	const Bakery* choose_repositioning_target(const Drone& drone) const;
};
