#include "simulation.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>

namespace {

constexpr double kRepositionEpsilon = 0.01;  ///< "already there" threshold

/// Build a fully-seeded Mersenne Twister. `std::random_device` provides
/// non-deterministic entropy (on Linux: /dev/urandom); feeding it through
/// `std::seed_seq` with eight draws fills mt19937's 19937-bit state
/// properly instead of relying on the warm-up pattern that a single
/// 32-bit seed produces. Called only from the main thread.
std::mt19937 make_random_engine() {
	std::random_device rd;
	std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
	return std::mt19937(seq);
}

/// Expected value of a discrete distribution given as (amount, prob) pairs.
double expected_value(const std::vector<std::pair<int, double>>& distribution) {
	double mean = 0.0;
	for (const auto& [amount, prob] : distribution) mean += amount * prob;
	return mean;
}

/// Gravity score = fullness / distance. Higher means more urgent to visit:
/// a bakery near its ceiling will waste production if no drone arrives, so
/// closeness * fullness wins over closeness alone.
double calculate_gravity_score(const Bakery& bakery, double distance) {
	if (bakery.capacity <= 0) {
		return -std::numeric_limits<double>::infinity();
	}
	const double fullness =
		(bakery.current_inventory + expected_value(bakery.production_distribution))
		/ static_cast<double>(bakery.capacity);
	return fullness / std::max(distance, 1.0);
}

/// Build a "go to bakery, pick up nothing" RouteNode used for repositioning.
RouteNode make_repositioning_hop(const Bakery& target) {
	return {RouteNodeType::BAKERY_PICKUP, target.pos, target.id,
	        /*customer_id*/ -1, /*bread_amount*/ 0, /*committed*/ false};
}

/// Bakery-side contention resolution: per bakery, sort intents by score
/// desc and serve each its full request until the bakery runs out, then
/// truncate / drop the rest. Mutates `bakeries[].current_inventory`.
void resolve_bakery_contention(
	std::vector<Intent>& intents,
	std::vector<Bakery>& bakeries,
	std::vector<Intent>& resolved,
	std::map<int, int>&  allocation)
{
	std::map<int, std::vector<Intent*>> per_bakery;
	for (auto& it : intents) per_bakery[it.bakery_id].push_back(&it);

	for (auto& [bakery_id, list] : per_bakery) {
		std::sort(list.begin(), list.end(),
			[](const Intent* a, const Intent* b) {
				return a->original_score > b->original_score;
			});

		int available = bakeries[bakery_id].current_inventory;
		for (Intent* intent : list) {
			const int give = std::min(intent->requested_bread_amount, available);
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

/// Walk the GRASP-optimized route and adjust node amounts to the resolved
/// allocations. Truncated intents shrink, dropped intents disappear, and
/// committed (already-executed) nodes pass through unchanged.
std::vector<RouteNode> apply_allocation_to_route(
	const std::vector<RouteNode>& route,
	const std::map<int, int>& allocation)
{
	std::vector<RouteNode> out;
	out.reserve(route.size());

	for (const RouteNode& node : route) {
		const bool is_new_customer_node =
			!node.committed && node.customer_id >= 0 &&
			(node.type == RouteNodeType::BAKERY_PICKUP ||
			 node.type == RouteNodeType::CUSTOMER_DELIVERY);

		if (!is_new_customer_node) {
			out.push_back(node);
			continue;
		}

		const auto it = allocation.find(node.customer_id);
		if (it == allocation.end()) {
			out.push_back(node);
			continue;
		}
		if (it->second <= 0) continue;  // dropped — skip pickup AND delivery

		RouteNode adjusted = node;
		adjusted.bread_amount = it->second;
		out.push_back(adjusted);
	}
	return out;
}

}  // namespace


Simulation::Simulation(const SimConfig& config)
	: config(config), rng(make_random_engine()),
	  next_customer_id(0), next_drone_id(0),
	  current_round(0), total_bread_delivered(0), total_customers_served(0) {}


/* ---------- validation ---------- */

/// Throw on configurations that can never finish. Subtler shortages (slow
/// drones, total supply < total demand) still surface only at runtime.
void Simulation::validate_config() const {
	if (config.customer_configs.empty()) {
		throw std::runtime_error(
			"Invalid config: customer_configs is empty — there is no one to deliver to.");
	}

	if (config.drone_template.capacity_min <= 0) {
		throw std::runtime_error(
			"Invalid config: drone_template.capacity_min must be > 0 "
			"(got " + std::to_string(config.drone_template.capacity_min) + ").");
	}
	if (config.drone_template.capacity_max < config.drone_template.capacity_min) {
		throw std::runtime_error(
			"Invalid config: drone_template.capacity_max ("
			+ std::to_string(config.drone_template.capacity_max)
			+ ") must be >= capacity_min ("
			+ std::to_string(config.drone_template.capacity_min) + ").");
	}

	auto can_supply = [](const BakeryConfig& bc) {
		if (bc.initial_inventory > 0) return true;
		for (const auto& [amount, prob] : bc.production_distribution) {
			if (amount > 0 && prob > 0.0) return true;
		}
		return false;
	};

	const bool any_supply = std::any_of(
		config.bakery_configs.begin(), config.bakery_configs.end(), can_supply);
	if (!any_supply) {
		throw std::runtime_error(
			"Invalid config: no bakery can supply bread. Every bakery has "
			"initial_inventory == 0 and a production_distribution with no "
			"positive-amount outcome at positive probability — the simulation "
			"would never finish.");
	}
}


/* ---------- fleet management ---------- */

Drone Simulation::spawn_drone() {
	const DroneTemplate& t = config.drone_template;
	std::uniform_real_distribution<double> velocity_dist(t.velocity_min, t.velocity_max);
	std::uniform_int_distribution<int>     capacity_dist(t.capacity_min, t.capacity_max);

	Drone d;
	d.id             = next_drone_id++;
	d.current_pos    = config.base_pos;
	d.velocity       = velocity_dist(rng);
	d.current_load   = 0;
	d.max_capacity   = capacity_dist(rng);
	d.route_progress = 0;
	d.is_idle        = true;
	return d;
}

/// Organic supply/demand spawning: spawn iff bread exists to be moved AND
/// the total fleet capacity is strictly less than total unassigned demand.
/// The condition is self-bounding — the fleet stabilizes at exactly
/// ceil(D_demand / drone.max_capacity) drones, so we need no headcount cap.
/// Per-customer urgency is intentionally NOT considered here: priority_weight
/// already steers the existing fleet through the GRASP score.
bool Simulation::should_spawn_drone() const {
	double D_demand = 0.0;
	for (const Customer& c : get_all_customers()) {
		if (assigned_customer_ids.count(c.id)) continue;
		D_demand += c.order_quantity;
	}

	double B_supply = 0.0;
	for (const Bakery& b : bakeries) B_supply += b.current_inventory;

	double C_fleet = 0.0;
	for (const Drone& d : drones) C_fleet += d.max_capacity;

	return (B_supply > 0.0) && (C_fleet < D_demand);
}

void Simulation::maybe_spawn_drone() {
	if (should_spawn_drone()) drones.push_back(spawn_drone());
}

Drone* Simulation::find_drone(int id) {
	for (auto& d : drones) if (d.id == id) return &d;
	return nullptr;
}


/* ---------- lifecycle ---------- */

void Simulation::initialize() {
	for (std::size_t i = 0; i < config.bakery_configs.size(); ++i) {
		const auto& bc = config.bakery_configs[i];
		bakeries.push_back({static_cast<int>(i), bc.pos,
		                    bc.initial_inventory, bc.capacity,
		                    bc.production_distribution});
	}

	delivery_graph.initialize(bakeries, config.base_pos);

	for (int i = 0; i < config.drone_template.initial_count; ++i) {
		drones.push_back(spawn_drone());
	}

	for (const auto& cc : config.customer_configs) {
		Customer c;
		c.id              = next_customer_id++;
		c.pos             = cc.pos;
		c.order_quantity  = cc.order_quantity;
		c.priority_weight = 1.0;
		customer_queue.push(c);
	}

	thread_pool = std::make_unique<ThreadPool>(config.thread_count);

	std::cout << "Simulation initialized:\n"
	          << "  Bakeries: "          << bakeries.size() << "\n"
	          << "  Drones: "            << drones.size() << "\n"
	          << "  Initial customers: " << customer_queue.size() << "\n"
	          << "  Grid: "              << config.grid_width << "x" << config.grid_height << "\n"
	          << "  Threads: "           << config.thread_count << "\n";
}

void Simulation::reset() {
	bakeries.clear();
	drones.clear();
	while (!customer_queue.empty()) customer_queue.pop();
	last_resolved_intents.clear();
	assigned_customer_ids.clear();
	served_this_round.clear();
	current_round          = 0;
	next_customer_id       = 0;
	next_drone_id          = 0;
	total_bread_delivered  = 0;
	total_customers_served = 0;
	rng = make_random_engine();
	initialize();
}

void Simulation::add_customer(double x, double y, int order_quantity,
                              const std::string& /*name*/) {
	Customer c;
	c.id              = next_customer_id++;
	c.pos             = {x, y};
	c.order_quantity  = order_quantity;
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


/* ---------- per-drone motion ---------- */

/// One velocity step toward the next route node. Pickups commit cargo
/// immediately; deliveries queue an event so the priority_queue mutation
/// stays on the main thread.
void Simulation::advance_drone(Drone& drone) {
	if (drone.planned_route.empty() ||
	    drone.route_progress >= static_cast<int>(drone.planned_route.size())) {
		drone.planned_route.clear();
		drone.route_progress = 0;
		drone.is_idle        = true;
		return;
	}

	drone.is_idle = false;
	RouteNode& target = drone.planned_route[drone.route_progress];
	const double dist = DeliveryGraph::compute_distance(drone.current_pos, target.pos);

	if (dist > drone.velocity) {
		const double ratio = drone.velocity / dist;
		drone.current_pos.x += (target.pos.x - drone.current_pos.x) * ratio;
		drone.current_pos.y += (target.pos.y - drone.current_pos.y) * ratio;
		return;
	}

	drone.current_pos = target.pos;
	if (target.type == RouteNodeType::BAKERY_PICKUP) {
		target.committed   = true;
		drone.current_load += target.bread_amount;
	} else if (target.type == RouteNodeType::CUSTOMER_DELIVERY) {
		drone.current_load -= target.bread_amount;
		drone.pending_deliveries.push_back({target.entity_id, target.bread_amount});
	}
	drone.route_progress++;
}

/// Drain queued delivery events into the customer queue. Single-threaded:
/// std::priority_queue is not thread-safe.
void Simulation::apply_delivery_events() {
	served_this_round.clear();

	std::map<int, int> delivered;
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
		const auto it = delivered.find(c.id);
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


/* ---------- pipeline ---------- */

/// Stage 1: bakery production + drone motion in parallel; post-step
/// bookkeeping (delivery events, priority increments) on the main thread.
void Simulation::stage1_state_update() {
	std::vector<std::future<void>> futures;

	// One RNG per bakery — workers must not share the master rng.
	std::vector<std::mt19937> bakery_rngs;
	bakery_rngs.reserve(bakeries.size());
	for (std::size_t i = 0; i < bakeries.size(); ++i) bakery_rngs.emplace_back(rng());

	for (std::size_t i = 0; i < bakeries.size(); ++i) {
		futures.push_back(thread_pool->submit([this, i, &bakery_rngs] {
			std::uniform_real_distribution<double> p(0.0, 1.0);
			const double roll = p(bakery_rngs[i]);
			double cum = 0.0;
			for (const auto& [amount, prob] : bakeries[i].production_distribution) {
				cum += prob;
				if (roll <= cum) {
					bakeries[i].current_inventory = std::min(
						bakeries[i].current_inventory + amount, bakeries[i].capacity);
					break;
				}
			}
		}));
	}

	for (std::size_t i = 0; i < drones.size(); ++i) {
		futures.push_back(thread_pool->submit([this, i] { advance_drone(drones[i]); }));
	}
	for (auto& f : futures) f.get();

	apply_delivery_events();

	// Customers not served this round get +priority_increment.
	std::vector<Customer> customers = get_all_customers();
	while (!customer_queue.empty()) customer_queue.pop();
	for (Customer c : customers) {
		if (!served_this_round.count(c.id)) c.priority_weight += config.priority_increment;
		customer_queue.push(c);
	}
}

/// Stages 2-3: parallel GRASP. The distance matrix is built once on the
/// main thread (no OMP, so the custom ThreadPool isn't oversubscribed);
/// each worker runs its share of iterations on private snapshots.
GraspSolution Simulation::stage2_3_assignment() {
	std::vector<Customer> customers;
	for (const Customer& c : get_all_customers()) {
		if (!assigned_customer_ids.count(c.id)) customers.push_back(c);
	}
	if (customers.empty()) return {};

	std::vector<Bakery> bakeries_snap = bakeries;
	std::vector<Drone>  drones_snap   = drones;

	DistanceCache dist_cache;
	dist_cache.build(bakeries_snap, customers, drones_snap, config.base_pos);

	const int total_iters = config.grasp_iterations;
	const int num_threads = std::min(config.thread_count, total_iters);

	std::vector<std::mt19937> thread_rngs;
	thread_rngs.reserve(num_threads);
	for (int i = 0; i < num_threads; ++i) thread_rngs.emplace_back(rng());

	struct ThreadResult {
		GraspSolution solution;
		double score = -1.0;
	};

	std::vector<std::future<ThreadResult>> futures;
	const int per_thread = total_iters / num_threads;
	const int extras     = total_iters % num_threads;

	for (int t = 0; t < num_threads; ++t) {
		const int my_iters = per_thread + (t < extras ? 1 : 0);
		futures.push_back(thread_pool->submit(
			[this, &customers, &bakeries_snap, &drones_snap,
			 &dist_cache, &thread_rngs, t, my_iters] {
				GraspSolver solver(delivery_graph, config.rcl_size, config.grasp_iterations);
				ThreadResult best;
				for (int i = 0; i < my_iters; ++i) {
					GraspSolution sol = solver.run_single_iteration(
						customers, drones_snap, bakeries_snap,
						config.base_pos, dist_cache, thread_rngs[t]);
					const double s = GraspSolver::evaluate_solution(sol.intents);
					if (s > best.score) {
						best.score    = s;
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
			best       = std::move(r.solution);
		}
	}
	return best;
}

/// Stage 4: resolve contention, stamp final amounts onto each drone's
/// route, then send any remaining idle drone toward a high-gravity bakery.
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
}


/* ---------- idle repositioning ---------- */

/// Pick the bakery with the highest Gravity Score from this drone's
/// current position. Returns nullptr if there are no bakeries.
const Bakery* Simulation::choose_repositioning_target(const Drone& drone) const {
	const Bakery* best = nullptr;
	double best_score  = -std::numeric_limits<double>::infinity();
	for (const Bakery& b : bakeries) {
		const double dist  = DeliveryGraph::compute_distance(drone.current_pos, b.pos);
		const double score = calculate_gravity_score(b, dist);
		if (score > best_score) {
			best_score = score;
			best       = &b;
		}
	}
	return best;
}

/// Send each truly-idle drone to its highest-gravity bakery so that
/// stochastic production isn't wasted on a full silo.
void Simulation::reposition_idle_drones(const std::set<int>& assigned_drone_ids) {
	if (bakeries.empty()) return;

	for (Drone& d : drones) {
		if (!d.is_idle || assigned_drone_ids.count(d.id)) continue;
		if (!d.planned_route.empty()) continue;

		const Bakery* target = choose_repositioning_target(d);
		if (!target) continue;

		const double dist = DeliveryGraph::compute_distance(d.current_pos, target->pos);
		if (dist < kRepositionEpsilon) continue;  // already at the target

		d.planned_route.push_back(make_repositioning_hop(*target));
		d.route_progress = 0;
		d.is_idle        = false;
	}
}


/* ---------- top level ---------- */

bool Simulation::step_round() {
	maybe_spawn_drone();
	stage1_state_update();
	GraspSolution solution = stage2_3_assignment();
	stage4_commit(solution);
	current_round++;

	if (customer_queue.empty()) {
		const bool all_idle = std::all_of(drones.begin(), drones.end(),
			[](const Drone& d) { return d.is_idle; });
		if (all_idle) return false;
	}
	return true;
}

void Simulation::run() {
	validate_config();
	initialize();

	std::cout << "\nStarting simulation\n";
	while (true) {
		const bool more = step_round();
		std::cout << "\r  Round " << current_round
		          << " | Customers: " << customer_queue.size()
		          << " | Drones: "    << drones.size()
		          << std::flush;
		if (!more) break;
	}
	std::cout << "\nAll customers served. Stopping at round "
	          << current_round << "\n";

	std::cout << "\nDone."
	          << " Bread delivered: "  << total_bread_delivered
	          << ", Customers served: " << total_customers_served << "\n";
}
