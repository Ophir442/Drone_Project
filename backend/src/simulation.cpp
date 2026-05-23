/**
 * @file simulation.cpp
 * @brief Simulation pipeline, fleet auto-scaling, and two-phase commit.
 *
 * See simulation.h for the per-round pipeline overview. This file groups
 * implementation into four sections:
 *   - file-scope helpers (RNG seeding, contention resolution, route stamping)
 *   - lifecycle  (constructor, validate_config, initialize, reset)
 *   - per-drone motion and delivery event drain
 *   - pipeline stages 1..4 plus the top-level run/step_round
 */

#include "simulation.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>

namespace {

constexpr double kRepositionEpsilon = 0.01;  ///< "already at the target" threshold
constexpr double kPriorityCeiling   = 1e12;  ///< Cap on priority_weight to prevent NaN drift

/**
 * @brief Build a fully-seeded Mersenne Twister.
 *
 * Both branches feed std::seed_seq exactly 256 bits of input so mt19937's
 * 19937-bit state expansion has wide entropy regardless of seed source:
 *   - Non-deterministic: 8 × 32-bit draws from std::random_device (/dev/urandom).
 *   - Deterministic:     8 × 32-bit words derived from @c cfg.seed via four
 *                        SplitMix64 iterations. A naive {seed_low, seed_high}
 *                        feed underseeds mt19937 (Vigna 2014) — early state
 *                        words become highly correlated for nearby seeds.
 *
 * Called only from the main thread.
 */
std::mt19937 make_random_engine(const SimConfig& cfg) {
	if (cfg.deterministic) {
		// SplitMix64: deterministic but avalanche-strong, the canonical way
		// to expand a single 64-bit seed into a wide initial state.
		auto splitmix = [](std::uint64_t& x) {
			x += 0x9E3779B97F4A7C15ULL;
			std::uint64_t z = x;
			z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
			z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
			return z ^ (z >> 31);
		};
		std::uint64_t s = cfg.seed;
		const std::uint64_t a = splitmix(s);
		const std::uint64_t b = splitmix(s);
		const std::uint64_t c = splitmix(s);
		const std::uint64_t d = splitmix(s);
		std::seed_seq seq{
			static_cast<std::uint32_t>(a),        static_cast<std::uint32_t>(a >> 32),
			static_cast<std::uint32_t>(b),        static_cast<std::uint32_t>(b >> 32),
			static_cast<std::uint32_t>(c),        static_cast<std::uint32_t>(c >> 32),
			static_cast<std::uint32_t>(d),        static_cast<std::uint32_t>(d >> 32),
		};
		return std::mt19937(seq);
	}
	std::random_device rd;
	std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
	return std::mt19937(seq);
}

/// Expected value of a discrete (amount, prob) distribution.
double expected_value(const std::vector<std::pair<int, double>>& distribution) {
	double mean = 0.0;
	for (const auto& [amount, prob] : distribution) mean += amount * prob;
	return mean;
}

/**
 * @brief Gravity score for an idle-drone repositioning target.
 *
 * Gravity = (current_inventory + expected_production) / capacity / max(distance, 1).
 *
 * Bakeries close to overflow attract idle drones — production that would
 * otherwise be discarded by the capacity clamp becomes deliverable. Tiebreak
 * toward closer targets.
 */
double calculate_gravity_score(const Bakery& bakery, double distance) {
	if (bakery.capacity <= 0) {
		return -std::numeric_limits<double>::infinity();
	}
	const double fullness =
		(bakery.current_inventory + expected_value(bakery.production_distribution))
		/ static_cast<double>(bakery.capacity);
	return fullness / std::max(distance, 1.0);
}

/// Construct a "go to bakery, pick up nothing" RouteNode for repositioning hops.
RouteNode make_repositioning_hop(const Bakery& target) {
	return {RouteNodeType::BAKERY_PICKUP, target.pos, target.id,
	        /*customer_id*/ -1, /*bread_amount*/ 0, /*committed*/ false};
}

/**
 * @brief Bakery-side phase-2 contention resolution.
 *
 * Group intents by bakery, sort each group by original_score descending, and
 * serve each intent its full request until that bakery runs out — at which
 * point remaining intents are truncated or dropped. Mutates the SHARED
 * bakery inventory (the authoritative copy) so subsequent rounds see post-state.
 *
 * @c allocation receives one entry per customer_id: the final granted
 * amount (0 = dropped). Only granted (give > 0) intents are pushed into
 * @c resolved.
 */
void resolve_bakery_contention(
	std::vector<Intent>& intents,
	std::vector<Bakery>& bakeries,
	std::vector<Intent>& resolved,
	std::unordered_map<int, int>& allocation)
{
	std::unordered_map<int, std::vector<Intent*>> per_bakery;
	per_bakery.reserve(bakeries.size());
	for (auto& it : intents) per_bakery[it.bakery_id].push_back(&it);

	// Per-bakery iteration order is now unspecified, but each per-bakery
	// group is independently sorted by original_score below — bakery
	// iteration order does not affect the resolution outcome.
	for (auto& [bakery_id, list] : per_bakery) {
		std::sort(list.begin(), list.end(),
			[](const Intent* a, const Intent* b) {
				return a->original_score > b->original_score;
			});

		int available = bakeries[bakery_id].current_inventory;
		assert(available >= 0);
		assert(available <= bakeries[bakery_id].capacity);

		for (Intent* intent : list) {
			const int give = std::min(intent->requested_bread_amount, available);
			assert(give >= 0);
			assert(give <= available);
			assert(give <= bakeries[bakery_id].capacity);   // Giant-Drone guard
			allocation[intent->customer_id] = give;
			if (give > 0) {
				available -= give;
				bakeries[bakery_id].current_inventory -= give;
				assert(bakeries[bakery_id].current_inventory >= 0);
				Intent r = *intent;
				r.requested_bread_amount = give;
				resolved.push_back(r);
			}
		}
	}
}

/**
 * @brief Rewrite a GRASP-optimized route to reflect post-contention amounts.
 *
 * Walks the planned route and, for each non-committed pickup/delivery whose
 * customer appears in @p allocation:
 *   - drops the node if the customer was fully dropped (allocation == 0),
 *   - else stamps the smaller allocated amount.
 *
 * Committed nodes pass through unchanged — already-executed pickups and
 * deliveries must never be rewritten retroactively.
 */
std::vector<RouteNode> apply_allocation_to_route(
	const std::vector<RouteNode>& route,
	const std::unordered_map<int, int>& allocation)
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

		// Allocation may shrink a node (truncated by contention), never grow
		// it — a larger amount would invent bread the resolver never approved.
		assert(it->second <= node.bread_amount);

		RouteNode adjusted = node;
		adjusted.bread_amount = it->second;
		out.push_back(adjusted);
	}
	return out;
}

}  // namespace


Simulation::Simulation(const SimConfig& config)
	: config(config), rng(make_random_engine(config)),
	  next_customer_id(0), next_drone_id(0),
	  current_round(0), total_bread_delivered(0), total_customers_served(0)
{
	// Single source of truth: validation happens at construction, so every
	// execution path (run(), HTTP server, /api/reset) is guaranteed to operate
	// on a config that won't NaN-propagate or divide by zero in the workers.
	validate_config();
}


/* ---------- validation ---------- */

/**
 * @brief Reject configurations that would crash, hang, or NaN-propagate.
 *
 * Each guard prevents a concrete failure mode previously catalogued in the
 * pessimistic-lens audits. Subtler shortages (slow drones overall, total
 * supply < total demand) still surface only at runtime — those are
 * legitimate stress scenarios, not config errors.
 */
void Simulation::validate_config() const {
	if (config.customer_configs.empty()) {
		throw std::runtime_error(
			"Invalid config: customer_configs is empty — there is no one to deliver to.");
	}
	if (config.bakery_configs.empty()) {
		throw std::runtime_error(
			"Invalid config: bakery_configs is empty — no producers exist.");
	}

	// ---- drone template ----
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
	if (config.drone_template.velocity_min <= 0.0) {
		throw std::runtime_error(
			"Invalid config: drone_template.velocity_min must be > 0 "
			"(got " + std::to_string(config.drone_template.velocity_min)
			+ "). A zero-velocity drone divides by zero in route timing "
			"and never advances physically.");
	}
	if (config.drone_template.velocity_max < config.drone_template.velocity_min) {
		throw std::runtime_error(
			"Invalid config: drone_template.velocity_max ("
			+ std::to_string(config.drone_template.velocity_max)
			+ ") must be >= velocity_min ("
			+ std::to_string(config.drone_template.velocity_min) + ").");
	}

	// ---- algorithm parameters (division-by-zero in stage2_3_assignment) ----
	if (config.thread_count <= 0) {
		throw std::runtime_error(
			"Invalid config: algorithm.thread_count must be >= 1 "
			"(got " + std::to_string(config.thread_count) + ").");
	}
	if (config.grasp_iterations <= 0) {
		throw std::runtime_error(
			"Invalid config: algorithm.grasp_iterations must be >= 1 "
			"(got " + std::to_string(config.grasp_iterations) + ").");
	}
	if (config.rcl_size <= 0) {
		throw std::runtime_error(
			"Invalid config: algorithm.rcl_size must be >= 1 "
			"(got " + std::to_string(config.rcl_size) + ").");
	}

	// ---- simulation parameters ----
	if (!std::isfinite(config.priority_increment) || config.priority_increment < 0.0) {
		throw std::runtime_error(
			"Invalid config: simulation.priority_increment must be finite and >= 0.");
	}

	// ---- bakeries: supply + probability sums ----
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

	for (std::size_t i = 0; i < config.bakery_configs.size(); ++i) {
		const auto& bc = config.bakery_configs[i];
		if (bc.capacity <= 0) {
			throw std::runtime_error(
				"Invalid config: bakery " + std::to_string(i)
				+ " has non-positive capacity (" + std::to_string(bc.capacity) + ").");
		}
		double prob_sum = 0.0;
		for (const auto& [amount, prob] : bc.production_distribution) {
			if (prob < 0.0 || prob > 1.0) {
				throw std::runtime_error(
					"Invalid config: bakery " + std::to_string(i)
					+ " has a production_distribution probability outside [0,1]: "
					+ std::to_string(prob) + ".");
			}
			prob_sum += prob;
		}
		if (!bc.production_distribution.empty() &&
		    std::abs(prob_sum - 1.0) > 1e-6) {
			throw std::runtime_error(
				"Invalid config: bakery " + std::to_string(i)
				+ " production_distribution probabilities sum to "
				+ std::to_string(prob_sum) + ", expected 1.0.");
		}
	}

	// ---- "Giant-Drone" deadlock prevention ----
	// A drone whose max payload exceeds *every* bakery's silo ceiling could
	// be assigned an order it can never fill in one stop, and naive code that
	// waits for the full pickup would deadlock. Runtime clamps in
	// build_candidates() defuse this when it happens — but defensive
	// programming says: refuse the config that makes it possible, so the
	// failure mode is impossible by construction.
	int max_bakery_capacity = 0;
	for (const BakeryConfig& bc : config.bakery_configs) {
		if (bc.capacity > max_bakery_capacity) max_bakery_capacity = bc.capacity;
	}
	if (config.drone_template.capacity_max > max_bakery_capacity) {
		throw std::runtime_error(
			"Invalid config: drone_template.capacity_max ("
			+ std::to_string(config.drone_template.capacity_max)
			+ ") exceeds the largest bakery capacity ("
			+ std::to_string(max_bakery_capacity)
			+ "). A spawned drone could be assigned an order no single bakery "
			"can fill — risks the \"Giant-Drone\" deadlock.");
	}

	// ---- customers: zero-order phantoms livelock the queue ----
	for (std::size_t i = 0; i < config.customer_configs.size(); ++i) {
		if (config.customer_configs[i].order_quantity <= 0) {
			throw std::runtime_error(
				"Invalid config: customer " + std::to_string(i)
				+ " has non-positive order_quantity ("
				+ std::to_string(config.customer_configs[i].order_quantity)
				+ "). A zero-order customer can never leave the priority queue.");
		}
	}
}


/* ---------- fleet management (auto-scaling) ---------- */

/**
 * @brief Spawn a single drone from the configured template distribution.
 *
 * Velocity and capacity are drawn from the template ranges. validate_config
 * has already established that velocity_min > 0 and capacity_min > 0, so the
 * spawned drone is guaranteed safe to schedule.
 */
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

	assert(d.velocity     > 0.0);   // belt-and-suspenders against validate_config drift
	assert(d.max_capacity > 0);
	return d;
}

/**
 * @brief Organic supply/demand drone spawning.
 *
 * Spawns iff bread exists to be moved AND total fleet capacity is strictly
 * less than total unassigned demand. The condition is self-bounding — the
 * fleet stabilizes at exactly ceil(D_demand / drone.max_capacity) drones,
 * so we need no headcount cap. Per-customer urgency is intentionally NOT
 * considered here: priority_weight already steers the existing fleet through
 * the GRASP score.
 */
bool Simulation::should_spawn_drone() const {
	double D_demand = 0.0;
	for (const Customer& c : customer_queue) {
		assert(c.order_quantity >= 0);
		if (assigned_customer_ids.count(c.id)) continue;
		D_demand += c.order_quantity;
	}

	double B_supply = 0.0;
	for (const Bakery& b : bakeries) {
		assert(b.current_inventory >= 0);
		assert(b.current_inventory <= b.capacity);
		B_supply += b.current_inventory;
	}

	double C_fleet = 0.0;
	for (const Drone& d : drones) {
		assert(d.max_capacity > 0);
		C_fleet += d.max_capacity;
	}

	assert(D_demand >= 0.0);
	assert(B_supply >= 0.0);
	assert(C_fleet >= 0.0);

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

	// INVARIANT: bakeries are stored with id == vector index. resolve_bakery_contention,
	// apply_allocation_to_route, and find_best_bakery all depend on this for O(1)
	// indexing. Future refactors that re-number bakeries MUST re-establish this.
	for (std::size_t i = 0; i < bakeries.size(); ++i) {
		assert(bakeries[i].id == static_cast<int>(i));
	}

	delivery_graph.initialize(bakeries, config.base_pos);

	for (int i = 0; i < config.drone_template.initial_count; ++i) {
		drones.push_back(spawn_drone());
	}

	customer_queue.reserve(config.customer_configs.size());
	for (const auto& cc : config.customer_configs) {
		Customer c;
		c.id              = next_customer_id++;
		c.pos             = cc.pos;
		c.order_quantity  = cc.order_quantity;
		c.priority_weight = 1.0;
		customer_queue.push_back(c);
	}

	thread_pool = std::make_unique<ThreadPool>(config.thread_count);
}

void Simulation::reset() {
	bakeries.clear();
	drones.clear();
	customer_queue.clear();
	last_resolved_intents.clear();
	assigned_customer_ids.clear();
	served_this_round.clear();
	current_round          = 0;
	next_customer_id       = 0;
	next_drone_id          = 0;
	total_bread_delivered  = 0;
	total_customers_served = 0;
	rng = make_random_engine(config);
	initialize();
}

void Simulation::add_customer(double x, double y, int order_quantity,
                              const std::string& /*name*/) {
	if (order_quantity <= 0) return;  // silently ignore phantom orders
	Customer c;
	c.id              = next_customer_id++;
	c.pos             = {x, y};
	c.order_quantity  = order_quantity;
	c.priority_weight = 1.0;
	customer_queue.push_back(c);
}

void Simulation::remove_customer(int customer_id) {
	customer_queue.erase(
		std::remove_if(customer_queue.begin(), customer_queue.end(),
			[customer_id](const Customer& c) { return c.id == customer_id; }),
		customer_queue.end());
}


/* ---------- per-drone motion ---------- */

/**
 * @brief One velocity-step toward the next route node.
 *
 * If the drone is within one step of the target, it arrives and either
 * commits a pickup (cargo += amount) or queues a delivery event (cargo -=
 * amount). Delivery events are drained on the main thread by
 * apply_delivery_events so the customer vector stays single-writer.
 *
 * Marking the just-executed node as @c committed protects it from later
 * mutation (allocation rewriting, 2-Opt reordering).
 */
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
		// Partial step: linearly interpolate toward the target.
		const double ratio = drone.velocity / dist;
		drone.current_pos.x += (target.pos.x - drone.current_pos.x) * ratio;
		drone.current_pos.y += (target.pos.y - drone.current_pos.y) * ratio;
		return;
	}

	drone.current_pos = target.pos;
	if (target.type == RouteNodeType::BAKERY_PICKUP) {
		// A pickup must never push the drone past its max payload.
		assert(target.bread_amount >= 0);
		assert(drone.current_load + target.bread_amount <= drone.max_capacity);
		target.committed   = true;
		drone.current_load += target.bread_amount;
	} else if (target.type == RouteNodeType::CUSTOMER_DELIVERY) {
		// A delivery must never undershoot zero — the drone must hold the cargo.
		assert(target.bread_amount >= 0);
		assert(drone.current_load >= target.bread_amount);
		// Commit the delivery so that (a) apply_allocation_to_route doesn't
		// rewrite an already-executed delivery's bread_amount when the
		// customer is re-queued for partial fulfilment, and (b) 2-Opt's
		// first_mutable correctly excludes it from the reorderable suffix.
		target.committed   = true;
		drone.current_load -= target.bread_amount;
		drone.pending_deliveries.push_back({target.entity_id, target.bread_amount});
	}
	assert(drone.current_load >= 0);
	assert(drone.current_load <= drone.max_capacity);
	drone.route_progress++;
}

/**
 * @brief Drain queued delivery events into the customer vector.
 *
 * Single-threaded: workers accumulate DeliveryEvents into per-drone buffers
 * during stage 1; this drain runs on the main thread after the futures join,
 * so the customer vector stays single-writer.
 *
 * Bookkeeping is in-place — no drain-and-rebuild of the container. Customers
 * whose order_quantity hits zero are erased via remove_if.
 */
void Simulation::apply_delivery_events() {
	served_this_round.clear();

	std::unordered_map<int, int> delivered;
	delivered.reserve(drones.size());
	for (Drone& d : drones) {
		for (const DeliveryEvent& ev : d.pending_deliveries) {
			assert(ev.bread_delivered >= 0);
			delivered[ev.customer_id] += ev.bread_delivered;
			total_bread_delivered     += ev.bread_delivered;
			assigned_customer_ids.erase(ev.customer_id);
			served_this_round.insert(ev.customer_id);
		}
		d.pending_deliveries.clear();
	}
	assert(total_bread_delivered >= 0);
	if (delivered.empty()) return;

	// In-place debit, then erase fully-served customers in a single sweep.
	for (Customer& c : customer_queue) {
		const auto it = delivered.find(c.id);
		if (it != delivered.end()) c.order_quantity -= it->second;
	}
	const std::size_t before = customer_queue.size();
	customer_queue.erase(
		std::remove_if(customer_queue.begin(), customer_queue.end(),
			[](const Customer& c) { return c.order_quantity <= 0; }),
		customer_queue.end());
	total_customers_served += static_cast<int>(before - customer_queue.size());
}


/* ---------- pipeline ---------- */

/**
 * @brief Stage 1: parallel state update.
 *
 * Bakery production (a stochastic sample from each bakery's distribution) and
 * drone motion (one velocity-step along each planned route) run concurrently
 * on the thread pool. Post-step bookkeeping — delivery event drain and
 * priority increment for unserved customers — runs on the main thread to
 * keep the customer vector single-writer.
 *
 * Each bakery worker gets its own RNG seeded off the main RNG; the master
 * std::mt19937 must not be shared across worker threads.
 */
void Simulation::stage1_state_update() {
	std::vector<std::future<void>> futures;

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
			assert(bakeries[i].current_inventory >= 0);
			assert(bakeries[i].current_inventory <= bakeries[i].capacity);
		}));
	}

	for (std::size_t i = 0; i < drones.size(); ++i) {
		futures.push_back(thread_pool->submit([this, i] { advance_drone(drones[i]); }));
	}
	for (auto& f : futures) f.get();

	apply_delivery_events();

	// Priority bookkeeping: customers not served this round get
	// +priority_increment, capped at kPriorityCeiling to prevent unbounded
	// growth (which could eventually produce NaN-tainted GRASP scores).
	// All unserved customers receive the SAME delta, so relative order is
	// preserved — no re-heapify or re-sort is needed here.
	for (Customer& c : customer_queue) {
		if (served_this_round.count(c.id)) continue;
		c.priority_weight += config.priority_increment;
		if (c.priority_weight > kPriorityCeiling) c.priority_weight = kPriorityCeiling;
	}
}

/**
 * @brief Stages 2-3: parallel GRASP.
 *
 * The DistanceCache is built once on the main thread so workers never touch
 * shared state when computing route times. The configured iteration budget
 * is split as evenly as possible across worker threads; each thread keeps
 * only its best-scoring solution, and the main thread reduces across them.
 *
 * GRASP needs customers in priority order (high priority first), so we sort
 * a one-shot snapshot here. The underlying customer_queue stays unsorted —
 * its only invariant is "contains all active customers."
 */
GraspSolution Simulation::stage2_3_assignment() {
	std::vector<Customer> customers;
	customers.reserve(customer_queue.size());
	for (const Customer& c : customer_queue) {
		if (!assigned_customer_ids.count(c.id)) customers.push_back(c);
	}
	if (customers.empty()) return {};

	// Customer::operator< is max-heap order (a < b iff a.priority < b.priority).
	// Sort descending = "b < a" comparator.
	std::sort(customers.begin(), customers.end(),
		[](const Customer& a, const Customer& b) { return b < a; });

	std::vector<Bakery> bakeries_snap = bakeries;
	std::vector<Drone>  drones_snap   = drones;

	DistanceCache dist_cache;
	dist_cache.build(bakeries_snap, customers, drones_snap, config.base_pos);

	const int total_iters = config.grasp_iterations;
	const int num_threads = std::min(config.thread_count, total_iters);
	if (num_threads <= 0) return {};  // belt-and-suspenders; validate_config blocks this

	std::vector<std::mt19937> thread_rngs;
	thread_rngs.reserve(num_threads);
	for (int i = 0; i < num_threads; ++i) thread_rngs.emplace_back(rng());

	struct ThreadResult {
		GraspSolution solution;
		double        score = -1.0;
	};

	std::vector<std::future<ThreadResult>> futures;
	const int per_thread = total_iters / num_threads;
	const int extras     = total_iters % num_threads;

	for (int t = 0; t < num_threads; ++t) {
		const int my_iters = per_thread + (t < extras ? 1 : 0);
		futures.push_back(thread_pool->submit(
			[this, &customers, &bakeries_snap, &drones_snap,
			 &dist_cache, &thread_rngs, t, my_iters] {
				GraspSolver solver(delivery_graph, config.rcl_size,
				                   config.grasp_iterations, dist_cache);
				ThreadResult best;
				for (int i = 0; i < my_iters; ++i) {
					GraspSolution sol = solver.run_single_iteration(
						customers, drones_snap, bakeries_snap,
						config.base_pos, thread_rngs[t]);
					const double s = sol.final_score;
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

/**
 * @brief Stage 4: two-phase commit + idle-drone repositioning.
 *
 * Phase 2 of two-phase commit:
 *   1. resolve_bakery_contention reconciles the per-thread intent promises
 *      against the real shared inventory, score-first, and returns the
 *      authoritative allocation.
 *   2. apply_allocation_to_route rewrites each drone's planned route to
 *      reflect the truncated/dropped amounts.
 *   3. Truly idle drones (not assigned any new work and with no remaining
 *      route) are sent toward the highest-gravity bakery so future
 *      production isn't wasted on a full silo.
 */
void Simulation::stage4_commit(GraspSolution& solution) {
	std::vector<Intent>          resolved;
	std::unordered_map<int, int> allocation;
	allocation.reserve(solution.intents.size());
	resolve_bakery_contention(solution.intents, bakeries, resolved, allocation);

	for (auto& [drone_id, route] : solution.drone_routes) {
		Drone* d = find_drone(drone_id);
		if (!d) continue;
		d->planned_route = apply_allocation_to_route(route, allocation);
		d->is_idle = d->planned_route.empty() ||
		             d->route_progress >= static_cast<int>(d->planned_route.size());
	}

	std::unordered_set<int> assigned_drone_ids;
	assigned_drone_ids.reserve(resolved.size());
	for (const Intent& it : resolved) {
		assigned_customer_ids.insert(it.customer_id);
		assigned_drone_ids.insert(it.drone_id);
	}

	reposition_idle_drones(assigned_drone_ids);

	last_resolved_intents = resolved;
}


/* ---------- idle repositioning ---------- */

/// Pick the bakery with the highest gravity score from this drone's current
/// position. Returns nullptr if there are no bakeries (defensive: validate_config
/// blocks that, but the read-only contract is worth preserving).
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

/// Send each truly-idle drone toward its highest-gravity bakery so that
/// stochastic production isn't wasted on a full silo.
void Simulation::reposition_idle_drones(const std::unordered_set<int>& assigned_drone_ids) {
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

/**
 * @brief Run a single round of the four-stage pipeline.
 *
 * Termination conditions, in order:
 *   1. All customers served AND all drones idle  → return false (normal end).
 *   2. No bakery has stock, none can ever produce, no drone is carrying
 *      cargo, AND customers remain → return false with a halt message.
 *      Without this clause the loop livelocks forever in an impossible state.
 *   3. Otherwise → return true.
 */
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

	// Dead-end detection: if every bakery is at zero inventory, no bakery
	// can ever produce a positive amount, and no drone is carrying cargo,
	// the remaining demand can never be satisfied. Continuing would livelock.
	int total_inventory = 0;
	int total_in_flight = 0;
	for (const Bakery& b : bakeries) total_inventory += b.current_inventory;
	for (const Drone&  d : drones)   total_in_flight += d.current_load;

	auto can_ever_produce = [](const Bakery& b) {
		for (const auto& [amount, prob] : b.production_distribution) {
			if (amount > 0 && prob > 0.0) return true;
		}
		return false;
	};
	const bool any_producer = std::any_of(
		bakeries.begin(), bakeries.end(), can_ever_produce);

	if (total_inventory == 0 && total_in_flight == 0 && !any_producer &&
	    !customer_queue.empty()) {
		std::cerr << "\n  Halting: no bread can ever reach "
		          << customer_queue.size() << " remaining customer(s).\n";
		return false;
	}

	return true;
}

void Simulation::run() {
	initialize();
	while (step_round()) {}
}
