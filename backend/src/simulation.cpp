#include "simulation.hpp"
#include <iostream>
#include <algorithm>
#include <map>

namespace {

// Resolve per-bakery contention. Sorts intents by score and gives each one
// what it asked for; if the bakery runs out, later intents are truncated or
// dropped. Mutates the bakery inventory and writes the per-customer outcome
// into `allocation` (0 means dropped).
void resolve_bakery_contention(
	std::vector<Intent>& intents,
	std::vector<Bakery>& bakeries,
	std::vector<Intent>& resolved,
	std::map<int, int>& allocation)
{
	std::map<int, std::vector<Intent*>> per_bakery;
	for (auto& it : intents) {
		per_bakery[it.bakery_id].push_back(&it);
	}

	for (auto& [bakery_id, list] : per_bakery) {
		std::sort(list.begin(), list.end(),
			[](const Intent* a, const Intent* b) {
				return a->original_score > b->original_score;
			});

		int available = bakeries[bakery_id].current_inventory;
		for (Intent* intent : list) {
			int give = std::min(intent->requested_bread_amount, available);
			allocation[intent->customer_id] = give;
			if (give > 0) {
				available -= give;
				bakeries[bakery_id].current_inventory -= give;
				Intent r = *intent;
				r.requested_bread_amount = give;
				resolved.push_back(r);
			}
		}
	}
}

// Walk the GRASP-optimized route and adjust nodes for the commit's
// allocations: nodes whose intent was truncated get a smaller bread_amount,
// nodes whose intent was dropped are removed. Pre-existing nodes from prior
// rounds (not in `allocation`) and non-customer nodes pass through unchanged.
std::vector<RouteNode> apply_allocation_to_route(
	const std::vector<RouteNode>& route,
	const std::map<int, int>& allocation)
{
	std::vector<RouteNode> out;
	out.reserve(route.size());

	for (const RouteNode& node : route) {
		bool is_new_customer_node =
			!node.committed && node.customer_id >= 0 &&
			(node.type == RouteNodeType::BAKERY_PICKUP ||
			 node.type == RouteNodeType::CUSTOMER_DELIVERY);

		if (!is_new_customer_node) {
			out.push_back(node);
			continue;
		}

		auto it = allocation.find(node.customer_id);
		if (it == allocation.end()) {
			// Carried over from a prior round; not part of this round's intents.
			out.push_back(node);
			continue;
		}

		if (it->second <= 0) continue; // intent dropped — skip pickup AND delivery

		RouteNode adjusted = node;
		adjusted.bread_amount = it->second;
		out.push_back(adjusted);
	}
	return out;
}

} // namespace


Simulation::Simulation(const SimConfig& config)
	: config(config), rng(42), next_customer_id(0), next_drone_id(0),
	  current_round(0), total_bread_delivered(0), total_customers_served(0) {}

Drone Simulation::spawn_drone(const DroneConfig& dc) {
	Drone d;
	d.id = next_drone_id++;
	d.current_pos = config.base_pos;
	d.velocity = dc.velocity;
	d.current_load = 0;
	d.max_capacity = dc.capacity;
	d.route_progress = 0;
	d.is_idle = true;
	return d;
}

Drone* Simulation::find_drone(int id) {
	for (auto& d : drones) {
		if (d.id == id) return &d;
	}
	return nullptr;
}

void Simulation::initialize() {
	for (size_t i = 0; i < config.bakery_configs.size(); ++i) {
		const auto& bc = config.bakery_configs[i];
		Bakery b;
		b.id = static_cast<int>(i);
		b.pos = bc.pos;
		b.capacity = bc.capacity;
		b.current_inventory = bc.initial_inventory;
		b.production_distribution = bc.production_distribution;
		bakeries.push_back(b);
	}

	delivery_graph.initialize(bakeries, config.base_pos);

	for (const auto& dc : config.drone_configs) {
		drones.push_back(spawn_drone(dc));
	}

	for (size_t i = 0; i < config.customer_configs.size(); ++i) {
		const auto& cc = config.customer_configs[i];
		Customer c;
		c.id = next_customer_id++;
		c.pos = cc.pos;
		c.order_quantity = cc.order_quantity;
		c.priority_weight = 1.0;
		customer_queue.push(c);
	}

	thread_pool = std::make_unique<ThreadPool>(config.thread_count);
	logger = std::make_unique<StateLogger>("output/state_log.json");

	std::cout << "Simulation initialized:\n"
		<< "  Bakeries: "          << bakeries.size() << "\n"
		<< "  Drones: "            << drones.size() << "\n"
		<< "  Initial customers: " << customer_queue.size() << "\n"
		<< "  Grid: "              << config.grid_width << "x" << config.grid_height << "\n"
		<< "  Threads: "           << config.thread_count << std::endl;
}

void Simulation::reset() {
	bakeries.clear();
	drones.clear();
	while (!customer_queue.empty()) customer_queue.pop();
	last_resolved_intents.clear();
	assigned_customer_ids.clear();
	served_this_round.clear();
	current_round = 0;
	next_customer_id = 0;
	next_drone_id = 0;
	total_bread_delivered = 0;
	total_customers_served = 0;
	rng.seed(42);
	initialize();
}

void Simulation::add_customer(double x, double y, int order_quantity, const std::string&) {
	Customer c;
	c.id = next_customer_id++;
	c.pos = {x, y};
	c.order_quantity = order_quantity;
	c.priority_weight = 1.0;
	customer_queue.push(c);
}

void Simulation::remove_customer(int customer_id) {
	std::vector<Customer> kept;
	while (!customer_queue.empty()) {
		Customer c = customer_queue.top();
		customer_queue.pop();
		if (c.id != customer_id) kept.push_back(c);
	}
	for (const Customer& c : kept) customer_queue.push(c);
}

std::vector<Customer> Simulation::get_all_customers() const {
	std::vector<Customer> out;
	auto copy = customer_queue;
	while (!copy.empty()) {
		out.push_back(copy.top());
		copy.pop();
	}
	return out;
}

// One velocity step toward the next route node. On arrival, mark a pickup
// committed (cargo loaded) or queue a delivery event; deliveries are applied
// sequentially later to keep this function thread-safe on shared state.
void Simulation::advance_drone(Drone& drone) {
	if (drone.planned_route.empty() ||
	    drone.route_progress >= static_cast<int>(drone.planned_route.size())) {
		drone.planned_route.clear();
		drone.route_progress = 0;
		drone.is_idle = true;
		return;
	}

	drone.is_idle = false;
	RouteNode& target = drone.planned_route[drone.route_progress];
	double dist = DeliveryGraph::compute_distance(drone.current_pos, target.pos);

	if (dist > drone.velocity) {
		double ratio = drone.velocity / dist;
		drone.current_pos.x += (target.pos.x - drone.current_pos.x) * ratio;
		drone.current_pos.y += (target.pos.y - drone.current_pos.y) * ratio;
		return;
	}

	drone.current_pos = target.pos;
	if (target.type == RouteNodeType::BAKERY_PICKUP) {
		target.committed = true;
		drone.current_load += target.bread_amount;
	} else if (target.type == RouteNodeType::CUSTOMER_DELIVERY) {
		drone.current_load -= target.bread_amount;
		drone.pending_deliveries.push_back({target.entity_id, target.bread_amount});
	}
	drone.route_progress++;
}

// Drains every drone's deferred delivery events and applies them to the
// shared customer queue. Sequential by design — the priority queue is not
// thread-safe.
void Simulation::apply_delivery_events() {
	served_this_round.clear();

	std::map<int, int> delivered; // customer_id -> bread received this round
	for (Drone& d : drones) {
		for (const DeliveryEvent& ev : d.pending_deliveries) {
			delivered[ev.customer_id] += ev.bread_delivered;
			total_bread_delivered     += ev.bread_delivered;
			assigned_customer_ids.erase(ev.customer_id);
			served_this_round.insert(ev.customer_id);
		}
		d.pending_deliveries.clear();
	}
	if (delivered.empty()) return;

	std::vector<Customer> kept;
	while (!customer_queue.empty()) {
		Customer c = customer_queue.top();
		customer_queue.pop();
		auto it = delivered.find(c.id);
		if (it != delivered.end()) {
			c.order_quantity -= it->second;
			if (c.order_quantity <= 0) {
				total_customers_served++;
				continue;
			}
		}
		kept.push_back(c);
	}
	for (const Customer& c : kept) customer_queue.push(c);
}

// Stage 1: bakery production and drone movement run in parallel; delivery
// events and customer-priority bookkeeping are applied sequentially after.
void Simulation::stage1_state_update() {
	std::vector<std::future<void>> futures;

	// One RNG per bakery so worker threads don't share the master rng.
	std::vector<std::mt19937> bakery_rngs;
	for (size_t i = 0; i < bakeries.size(); ++i) bakery_rngs.emplace_back(rng());

	for (size_t i = 0; i < bakeries.size(); ++i) {
		futures.push_back(thread_pool->submit([this, i, &bakery_rngs] {
			std::uniform_real_distribution<double> p(0.0, 1.0);
			double roll = p(bakery_rngs[i]);
			double cum = 0.0;
			for (const auto& [amount, prob] : bakeries[i].production_distribution) {
				cum += prob;
				if (roll <= cum) {
					bakeries[i].current_inventory =
						std::min(bakeries[i].current_inventory + amount,
								 bakeries[i].capacity);
					break;
				}
			}
		}));
	}

	for (size_t i = 0; i < drones.size(); ++i) {
		futures.push_back(thread_pool->submit([this, i] { advance_drone(drones[i]); }));
	}
	for (auto& f : futures) f.get();

	apply_delivery_events();

	// Per the spec: customers not served this round get +priority_increment.
	std::vector<Customer> customers = get_all_customers();
	while (!customer_queue.empty()) customer_queue.pop();
	for (Customer c : customers) {
		if (!served_this_round.count(c.id)) c.priority_weight += config.priority_increment;
		customer_queue.push(c);
	}
}

// Stages 2 and 3: parallel GRASP. Each thread runs its share of iterations
// over a private snapshot and returns the highest-scoring solution it found.
// We then pick the best across threads.
GraspSolution Simulation::stage2_3_assignment() {
	std::vector<Customer> customers;
	for (const Customer& c : get_all_customers()) {
		if (!assigned_customer_ids.count(c.id)) customers.push_back(c);
	}
	if (customers.empty()) return {};

	std::vector<Bakery> bakeries_snap = bakeries;
	std::vector<Drone>  drones_snap   = drones;

	int total_iters = config.grasp_iterations;
	int num_threads = std::min(config.thread_count, total_iters);

	std::vector<std::mt19937> thread_rngs;
	for (int i = 0; i < num_threads; ++i) thread_rngs.emplace_back(rng());

	struct ThreadResult {
		GraspSolution solution;
		double score = -1.0;
	};

	std::vector<std::future<ThreadResult>> futures;
	int per_thread = total_iters / num_threads;
	int extras     = total_iters % num_threads;

	for (int t = 0; t < num_threads; ++t) {
		int my_iters = per_thread + (t < extras ? 1 : 0);
		futures.push_back(thread_pool->submit(
			[this, &customers, &bakeries_snap, &drones_snap, &thread_rngs, t, my_iters] {
				GraspSolver solver(delivery_graph, config.rcl_size, config.grasp_iterations);
				ThreadResult best;
				for (int i = 0; i < my_iters; ++i) {
					GraspSolution sol = solver.run_single_iteration(
						customers, drones_snap, bakeries_snap, config.base_pos, thread_rngs[t]);
					double s = GraspSolver::evaluate_solution(sol.intents);
					if (s > best.score) {
						best.score = s;
						best.solution = std::move(sol);
					}
				}
				return best;
			}));
	}

	GraspSolution best;
	double best_score = -1.0;
	for (auto& f : futures) {
		ThreadResult r = f.get();
		if (r.score > best_score) {
			best_score = r.score;
			best = std::move(r.solution);
		}
	}
	return best;
}

// Stage 4: synchronized commit. Two steps — resolve bakery contention by
// score, then write the 2-Opt-optimized routes back to the real drones with
// the resolved bread_amounts applied.
void Simulation::stage4_commit(GraspSolution& solution) {
	std::vector<Intent> resolved;
	std::map<int, int>  allocation;
	resolve_bakery_contention(solution.intents, bakeries, resolved, allocation);

	for (auto& [drone_id, route] : solution.drone_routes) {
		Drone* d = find_drone(drone_id);
		if (!d) continue;
		d->planned_route = apply_allocation_to_route(route, allocation);
		d->is_idle = d->planned_route.empty() ||
		             d->route_progress >= static_cast<int>(d->planned_route.size());
	}

	std::set<int> assigned_drone_ids;
	for (const Intent& it : resolved) {
		assigned_customer_ids.insert(it.customer_id);
		assigned_drone_ids.insert(it.drone_id);
	}

	reposition_idle_drones(assigned_drone_ids);

	last_resolved_intents = resolved;
	logger->log_round(current_round, bakeries, customer_queue, drones,
					  config.base_pos, resolved);
}

// Per the spec: an idle drone with no mission may move toward a bakery to be
// better positioned for future orders. We pick the one with the most
// expected supply (current stock + average production rate).
void Simulation::reposition_idle_drones(const std::set<int>& assigned_drone_ids) {
	if (bakeries.empty()) return;

	int target_id = 0;
	double best = -1.0;
	for (const Bakery& b : bakeries) {
		double expected = 0.0;
		for (const auto& [amount, prob] : b.production_distribution) {
			expected += amount * prob;
		}
		double score = b.current_inventory + expected;
		if (score > best) {
			best = score;
			target_id = b.id;
		}
	}
	const Position& target_pos = bakeries[target_id].pos;

	for (Drone& d : drones) {
		if (!d.is_idle || assigned_drone_ids.count(d.id)) continue;
		if (!d.planned_route.empty()) continue;
		if (DeliveryGraph::compute_distance(d.current_pos, target_pos) < 0.01) continue;

		RouteNode hop;
		hop.type         = RouteNodeType::BAKERY_PICKUP;
		hop.pos          = target_pos;
		hop.entity_id    = target_id;
		hop.customer_id  = -1; // not tied to any customer
		hop.bread_amount = 0;  // no actual pickup, just movement
		hop.committed    = false;

		d.planned_route.push_back(hop);
		d.route_progress = 0;
		d.is_idle = false;
	}
}

bool Simulation::step_round() {
	if (current_round >= config.max_rounds) return false;

	stage1_state_update();
	GraspSolution solution = stage2_3_assignment();
	stage4_commit(solution);
	current_round++;

	if (customer_queue.empty()) {
		bool all_idle = std::all_of(drones.begin(), drones.end(),
			[](const Drone& d) { return d.is_idle; });
		if (all_idle) return false;
	}
	return true;
}

void Simulation::run() {
	initialize();

	std::cout << "\nStarting simulation\n";
	for (int round = 0; round < config.max_rounds; ++round) {
		std::cout << "\r  Round " << (round + 1) << "/" << config.max_rounds
			<< " | Customers: " << customer_queue.size()
			<< " | Drones: "    << drones.size()
			<< std::flush;
		if (!step_round()) {
			std::cout << "\nAll customers served. Stopping at round " << (round + 1) << "\n";
			break;
		}
	}

	std::cout << "\nDone."
		<< " Bread delivered: "  << total_bread_delivered
		<< ", Customers served: " << total_customers_served << "\n";
	logger->flush();
}
