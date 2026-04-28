#pragma once

#include "types.hpp"
#include "distance.hpp"
#include "grasp.hpp"
#include "thread_pool.hpp"
#include "state_logger.hpp"
#include <vector>
#include <queue>
#include <random>
#include <set>

class Simulation {
public:
	explicit Simulation(const SimConfig& config);

	void run();

	void initialize();
	void reset();
	bool step_round();

	void add_customer(double x, double y, int order_quantity, const std::string& name = "");
	void remove_customer(int customer_id);

	const std::vector<Bakery>& get_bakeries() const { return bakeries; }
	const std::vector<Drone>&  get_drones()   const { return drones; }
	std::vector<Customer>      get_all_customers() const;
	int get_round() const { return current_round; }
	const SimConfig& get_config() const { return config; }
	const Position&  get_base_pos() const { return config.base_pos; }
	const std::vector<Intent>& get_last_intents() const { return last_resolved_intents; }
	int get_total_delivered() const { return total_bread_delivered; }
	int get_total_customers_served() const { return total_customers_served; }

private:
	SimConfig config;

	std::vector<Bakery>            bakeries;
	std::priority_queue<Customer>  customer_queue;
	std::vector<Drone>             drones;
	DeliveryGraph                  delivery_graph;

	std::unique_ptr<ThreadPool>  thread_pool;
	std::unique_ptr<StateLogger> logger;
	std::mt19937                 rng;

	int next_customer_id;
	int next_drone_id;
	int current_round;
	int total_bread_delivered;
	int total_customers_served;

	std::vector<Intent> last_resolved_intents;
	std::set<int>       assigned_customer_ids; // customers with a drone already en route
	std::set<int>       served_this_round;     // customers who got bread this round

	// The four pipeline stages.
	void stage1_state_update();
	GraspSolution stage2_3_assignment();
	void stage4_commit(GraspSolution& solution);

	// Helpers used by the stages.
	void  advance_drone(Drone& drone);
	void  apply_delivery_events();
	void  reposition_idle_drones(const std::set<int>& assigned_drone_ids);
	Drone spawn_drone(const DroneConfig& dc);
	Drone* find_drone(int id);
};
