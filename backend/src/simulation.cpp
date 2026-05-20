#include "simulation.h"
#include <iostream>
#include <algorithm>
#include <limits>
#include <map>

using namespace std;

namespace {

// Per-bakery contention: sort intents by score, give each one what it asked
// for, truncate or drop later ones if the bakery runs out.
void resolve_bakery_contention(
	vector<Intent>& intents,
	vector<Bakery>& bakeries,
	vector<Intent>& resolved,
	map<int, int>& allocation)
{
	map<int, vector<Intent*>> per_bakery;
	for (auto& it : intents) per_bakery[it.bakery_id].push_back(&it);

	for (auto& [bakery_id, list] : per_bakery) {
		sort(list.begin(), list.end(),
			[](const Intent* a, const Intent* b) {
				return a->original_score > b->original_score;
			});

		int available = bakeries[bakery_id].current_inventory;
		for (Intent* intent : list) {
			int give = min(intent->requested_bread_amount, available);
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

// Walk the GRASP-optimized route and adjust nodes to match the committed
// allocations. Truncated intents shrink, dropped intents are removed, and
// nodes carried over from earlier rounds pass through unchanged.
vector<RouteNode> apply_allocation_to_route(
	const vector<RouteNode>& route,
	const map<int, int>& allocation)
{
	vector<RouteNode> out;
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
			out.push_back(node);
			continue;
		}

		if (it->second <= 0) continue; // dropped — skip pickup AND delivery

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

Drone Simulation::spawn_drone() {
	const DroneTemplate& t = config.drone_template;
	uniform_real_distribution<double> velocity_dist(t.velocity_min, t.velocity_max);

	Drone d;
	d.id             = next_drone_id++;
	d.current_pos    = config.base_pos;
	d.velocity       = velocity_dist(rng);
	d.current_load   = 0;
	d.max_capacity   = t.capacity;
	d.route_progress = 0;
	d.is_idle        = true;
	return d;
}

// Grow the fleet only when there's unmet demand and every drone is busy.
// Capped so we never have more drones than customers waiting — that's the
// "excessive" guard.
void Simulation::maybe_spawn_drone() {
	int unassigned = 0;
	for (const Customer& c : get_all_customers()) {
		if (!assigned_customer_ids.count(c.id)) ++unassigned;
	}
	if (unassigned == 0) return;

	for (const Drone& d : drones) {
		if (d.is_idle) return; // existing fleet has spare capacity
	}
	if (static_cast<int>(drones.size()) >= unassigned) return;

	drones.push_back(spawn_drone());
}

Drone* Simulation::find_drone(int id) {
	for (auto& d : drones) if (d.id == id) return &d;
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

	for (int i = 0; i < config.drone_template.initial_count; ++i) {
		drones.push_back(spawn_drone());
	}

	for (const auto& cc : config.customer_configs) {
		Customer c;
		c.id = next_customer_id++;
		c.pos = cc.pos;
		c.order_quantity = cc.order_quantity;
		c.priority_weight = 1.0;
		customer_queue.push(c);
	}

	thread_pool = make_unique<ThreadPool>(config.thread_count);

	cout << "Simulation initialized:\n"
	     << "  Bakeries: "          << bakeries.size() << "\n"
	     << "  Drones: "            << drones.size() << "\n"
	     << "  Initial customers: " << customer_queue.size() << "\n"
	     << "  Grid: "              << config.grid_width << "x" << config.grid_height << "\n"
	     << "  Threads: "           << config.thread_count << endl;
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

void Simulation::add_customer(double x, double y, int order_quantity, const string&) {
	Customer c;
	c.id = next_customer_id++;
	c.pos = {x, y};
	c.order_quantity = order_quantity;
	c.priority_weight = 1.0;
	customer_queue.push(c);
}

void Simulation::remove_customer(int customer_id) {
	vector<Customer> kept;
	while (!customer_queue.empty()) {
		Customer c = customer_queue.top();
		customer_queue.pop();
		if (c.id != customer_id) kept.push_back(c);
	}
	for (const Customer& c : kept) customer_queue.push(c);
}

vector<Customer> Simulation::get_all_customers() const {
	vector<Customer> out;
	auto copy = customer_queue;
	while (!copy.empty()) {
		out.push_back(copy.top());
		copy.pop();
	}
	return out;
}

// One velocity step toward the next route node. On arrival, mark a pickup
// committed (cargo loaded) or queue a delivery event. Deliveries are queued
// here and applied later in apply_delivery_events to keep this thread-safe.
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

// Sequential — the priority queue isn't thread-safe.
void Simulation::apply_delivery_events() {
	served_this_round.clear();

	map<int, int> delivered;
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

	vector<Customer> kept;
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

// Stage 1: bakery production and drone movement run in parallel; bookkeeping
// (delivery events, priority increments) is applied sequentially after.
void Simulation::stage1_state_update() {
	vector<future<void>> futures;

	// One RNG per bakery so worker threads don't share the master rng.
	vector<mt19937> bakery_rngs;
	bakery_rngs.reserve(bakeries.size());
	for (size_t i = 0; i < bakeries.size(); ++i) bakery_rngs.emplace_back(rng());

	for (size_t i = 0; i < bakeries.size(); ++i) {
		futures.push_back(thread_pool->submit([this, i, &bakery_rngs] {
			uniform_real_distribution<double> p(0.0, 1.0);
			double roll = p(bakery_rngs[i]);
			double cum = 0.0;
			for (const auto& [amount, prob] : bakeries[i].production_distribution) {
				cum += prob;
				if (roll <= cum) {
					bakeries[i].current_inventory =
						min(bakeries[i].current_inventory + amount, bakeries[i].capacity);
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

	// Spec: customers not served this round get +priority_increment.
	vector<Customer> customers = get_all_customers();
	while (!customer_queue.empty()) customer_queue.pop();
	for (Customer c : customers) {
		if (!served_this_round.count(c.id)) c.priority_weight += config.priority_increment;
		customer_queue.push(c);
	}
}

// Stages 2-3: parallel GRASP. Each thread runs its share of iterations on a
// private snapshot and returns its best solution; we pick the best overall.
GraspSolution Simulation::stage2_3_assignment() {
	vector<Customer> customers;
	for (const Customer& c : get_all_customers()) {
		if (!assigned_customer_ids.count(c.id)) customers.push_back(c);
	}
	if (customers.empty()) return {};

	vector<Bakery> bakeries_snap = bakeries;
	vector<Drone>  drones_snap   = drones;

	int total_iters = config.grasp_iterations;
	int num_threads = min(config.thread_count, total_iters);

	vector<mt19937> thread_rngs;
	thread_rngs.reserve(num_threads);
	for (int i = 0; i < num_threads; ++i) thread_rngs.emplace_back(rng());

	struct ThreadResult {
		GraspSolution solution;
		double score = -1.0;
	};

	vector<future<ThreadResult>> futures;
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
						best.solution = move(sol);
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
			best = move(r.solution);
		}
	}
	return best;
}

// Stage 4: synchronized commit. Resolve bakery contention by score, then
// write the optimized routes back to the real drones with bread_amounts
// adjusted to the resolved allocations.
void Simulation::stage4_commit(GraspSolution& solution) {
	vector<Intent> resolved;
	map<int, int>  allocation;
	resolve_bakery_contention(solution.intents, bakeries, resolved, allocation);

	for (auto& [drone_id, route] : solution.drone_routes) {
		Drone* d = find_drone(drone_id);
		if (!d) continue;
		d->planned_route = apply_allocation_to_route(route, allocation);
		d->is_idle = d->planned_route.empty() ||
		             d->route_progress >= static_cast<int>(d->planned_route.size());
	}

	set<int> assigned_drone_ids;
	for (const Intent& it : resolved) {
		assigned_customer_ids.insert(it.customer_id);
		assigned_drone_ids.insert(it.drone_id);
	}

	reposition_idle_drones(assigned_drone_ids);

	last_resolved_intents = resolved;
}

// Each idle drone repositions to its own nearest bakery. Picking the
// globally best-stocked bakery for everyone caused all idle drones to swarm
// the same spot; nearest-neighbour spreads the fleet across the map.
void Simulation::reposition_idle_drones(const set<int>& assigned_drone_ids) {
	if (bakeries.empty()) return;

	for (Drone& d : drones) {
		if (!d.is_idle || assigned_drone_ids.count(d.id)) continue;
		if (!d.planned_route.empty()) continue;

		int closest_id = -1;
		double closest_dist = numeric_limits<double>::max();
		for (const Bakery& b : bakeries) {
			double dist = DeliveryGraph::compute_distance(d.current_pos, b.pos);
			if (dist < closest_dist) {
				closest_dist = dist;
				closest_id   = b.id;
			}
		}
		if (closest_id < 0 || closest_dist < 0.01) continue; // already there

		const Bakery& target = bakeries[closest_id];
		RouteNode hop;
		hop.type         = RouteNodeType::BAKERY_PICKUP;
		hop.pos          = target.pos;
		hop.entity_id    = target.id;
		hop.customer_id  = -1; // not tied to any customer
		hop.bread_amount = 0;  // repositioning only
		hop.committed    = false;

		d.planned_route.push_back(hop);
		d.route_progress = 0;
		d.is_idle = false;
	}
}

bool Simulation::step_round() {
	if (current_round >= config.max_rounds) return false;

	maybe_spawn_drone();
	stage1_state_update();
	GraspSolution solution = stage2_3_assignment();
	stage4_commit(solution);
	current_round++;

	if (customer_queue.empty()) {
		bool all_idle = all_of(drones.begin(), drones.end(),
			[](const Drone& d) { return d.is_idle; });
		if (all_idle) return false;
	}
	return true;
}

void Simulation::run() {
	initialize();

	cout << "\nStarting simulation\n";
	for (int round = 0; round < config.max_rounds; ++round) {
		cout << "\r  Round " << (round + 1) << "/" << config.max_rounds
		     << " | Customers: " << customer_queue.size()
		     << " | Drones: "    << drones.size()
		     << flush;
		if (!step_round()) {
			cout << "\nAll customers served. Stopping at round " << (round + 1) << "\n";
			break;
		}
	}

	cout << "\nDone."
	     << " Bread delivered: "  << total_bread_delivered
	     << ", Customers served: " << total_customers_served << "\n";
}
