/**
 * @file grasp.cpp
 * @brief GRASP (Greedy Randomized Adaptive Search) for drone↔customer assignment.
 *
 * Each iteration constructs one feasible solution by, for every customer in
 * priority order:
 *   1. enumerate (drone, bakery, amount) candidates,
 *   2. take the top-rcl_size by score,
 *   3. sample uniformly from that Restricted Candidate List,
 *   4. extend the chosen drone's route, then improve with 2-Opt.
 *
 * Defensive programming is layered:
 *   - find_best_bakery POSTCONDITION: the returned amount is realisable in one stop.
 *   - build_candidates triple-clamp:  amount ≤ achievable, ≤ bakery.capacity, ≤ drone free.
 *   - is_route_valid CARGO-BALANCE:   per-customer pickup minus delivery never negative.
 *   - apply_two_opt PRECONDITION:     route is valid on entry.
 */

#include "grasp.h"

#include <algorithm>
#include <cassert>
#include <limits>

namespace {

constexpr double kMinRouteDeltaT = 0.001;  ///< Floor for the Δt divisor in score.
constexpr double kTwoOptEpsilon  = 1e-9;   ///< Minimum 2-Opt improvement to accept.

/// Bread already committed on a drone: current load + every scheduled pickup
/// remaining in its plan. Used to clamp new assignments against free capacity.
int future_load(const Drone& drone) {
	int load = drone.current_load;
	for (std::size_t i = drone.route_progress; i < drone.planned_route.size(); ++i) {
		if (drone.planned_route[i].type == RouteNodeType::BAKERY_PICKUP) {
			load += drone.planned_route[i].bread_amount;
		}
	}
	return load;
}

}  // namespace


GraspSolver::GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations)
	: graph(graph), rcl_size(rcl_size), iterations(iterations) {}


/* ---------- distance helpers ---------- */

/// Single dispatch point for "distance between two waypoints". Prefers the
/// per-round cache (matrix lookup) and falls back to the static delivery
/// graph (matrix or Euclidean) otherwise.
double GraspSolver::distance_between(const Position& a, int a_node,
                                     const Position& b, int b_node) const {
	return cache ? cache->lookup(a, a_node, b, b_node)
	             : graph.hybrid_distance(a, a_node, b, b_node);
}

/// Resolve the cache-or-graph node id of @p node. Customer nodes return -1
/// when no per-round cache is active, so the caller will Euclidean-fallback.
int GraspSolver::node_id_for(const RouteNode& node) const {
	if (cache) {
		switch (node.type) {
			case RouteNodeType::BAKERY_PICKUP:     return cache->bakery_node(node.entity_id);
			case RouteNodeType::BASE_RETURN:       return cache->base_node();
			case RouteNodeType::CUSTOMER_DELIVERY: return cache->customer_node(node.entity_id);
		}
		return -1;
	}
	switch (node.type) {
		case RouteNodeType::BAKERY_PICKUP:     return node.entity_id;
		case RouteNodeType::BASE_RETURN:       return graph.get_base_node();
		case RouteNodeType::CUSTOMER_DELIVERY: return -1;
	}
	return -1;
}

int GraspSolver::drone_start_node(const Drone& drone) const {
	return cache ? cache->drone_node(drone.id) : -1;
}

int GraspSolver::base_node_id() const {
	return cache ? cache->base_node() : graph.get_base_node();
}


/* ---------- route timing ---------- */

/// Sum hop distances along a route slice. When @p end_pos is non-null, the
/// final hop to that endpoint is appended (used to estimate the return leg
/// to the depot). Single hub for every distance-summation in this file.
double GraspSolver::path_distance(
	const Position& start_pos, int start_node,
	const std::vector<RouteNode>& route, std::size_t start_idx,
	const Position* end_pos, int end_node) const
{
	double total = 0.0;
	Position current = start_pos;
	int current_node = start_node;
	for (std::size_t i = start_idx; i < route.size(); ++i) {
		const RouteNode& rn = route[i];
		const int rn_node = node_id_for(rn);
		total += distance_between(current, current_node, rn.pos, rn_node);
		current = rn.pos;
		current_node = rn_node;
	}
	if (end_pos) {
		total += distance_between(current, current_node, *end_pos, end_node);
	}
	return total;
}

/// Estimated time-to-complete for a drone's *current* plan plus a return-to-base hop.
double GraspSolver::compute_route_time(const Drone& drone,
                                       const Position& base_pos) const {
	if (drone.planned_route.empty()) return 0.0;
	assert(drone.velocity > 0.0);
	const double d = path_distance(
		drone.current_pos, drone_start_node(drone),
		drone.planned_route, drone.route_progress,
		&base_pos, base_node_id());
	return d / drone.velocity;
}

/// Estimated time-to-complete for the drone's plan if we *appended* a
/// (pickup, delivery) pair for @p customer at @p bakery for @p amount bread.
/// Used to compute the marginal route cost Δt for the GRASP score.
double GraspSolver::compute_route_time_with_assignment(
	const Drone& drone, const Bakery& bakery, const Customer& customer,
	int amount, const Position& base_pos) const
{
	assert(drone.velocity > 0.0);
	std::vector<RouteNode> hypothetical = drone.planned_route;
	hypothetical.push_back(
		{RouteNodeType::BAKERY_PICKUP, bakery.pos, bakery.id, customer.id, amount, false});
	hypothetical.push_back(
		{RouteNodeType::CUSTOMER_DELIVERY, customer.pos, customer.id, customer.id, amount, false});

	const double d = path_distance(
		drone.current_pos, drone_start_node(drone),
		hypothetical, drone.route_progress,
		&base_pos, base_node_id());
	return d / drone.velocity;
}


/* ---------- bakery selection ---------- */

/**
 * @brief Choose the best bakery to source @p customer's order from.
 *
 * Preference order:
 *   1. The bakery that can cover the FULL order with the shortest extra
 *      route time. (Single-stop fulfilment is the cheapest dispatch.)
 *   2. If none can, fall back to the bakery with the highest current stock
 *      and accept a partial achievable_amount — the customer is re-queued
 *      and finishes filling on a later round.
 *
 * POSTCONDITION: the returned amount is physically realisable by a single
 * visit to the chosen bakery (i.e. ≤ current_inventory and ≤ capacity).
 */
BakeryAssignment GraspSolver::find_best_bakery(
	const Drone& drone, const Customer& customer,
	const std::vector<Bakery>& bakeries, const Position& base_pos) const
{
	BakeryAssignment best{-1, 0};
	double best_time = std::numeric_limits<double>::max();

	for (const Bakery& b : bakeries) {
		if (b.current_inventory < customer.order_quantity) continue;
		const double t = compute_route_time_with_assignment(
			drone, b, customer, customer.order_quantity, base_pos);
		if (t < best_time) {
			best_time = t;
			best.bakery_id = b.id;
			best.achievable_amount = customer.order_quantity;
		}
	}

	if (best.bakery_id == -1) {
		int max_inv = 0;
		for (const Bakery& b : bakeries) {
			if (b.current_inventory > max_inv) {
				max_inv = b.current_inventory;
				best.bakery_id = b.id;
				best.achievable_amount = b.current_inventory;
			}
		}
	}

	if (best.bakery_id >= 0) {
		const Bakery& b = bakeries[best.bakery_id];
		assert(best.achievable_amount >= 0);
		assert(best.achievable_amount <= b.current_inventory);
		assert(best.achievable_amount <= b.capacity);
	}
	return best;
}

/**
 * @brief Enumerate every (drone, bakery, amount) option for one customer.
 *
 * The score is  (priority_weight * fulfilment_ratio) / Δroute_time, so:
 *   - waitier customers (higher priority_weight) attract more drones,
 *   - bigger pickups beat smaller ones for the same trip cost,
 *   - small detours beat large ones.
 *
 * Order splitting keeps a candidate alive whenever the drone has *any*
 * remaining free capacity, even if the bakery could supply more — partial
 * fulfilment is always preferable to dropping a customer entirely.
 */
std::vector<GraspSolver::Candidate> GraspSolver::build_candidates(
	const Customer& customer,
	const std::vector<Drone>& drones,
	const std::vector<Bakery>& bakeries,
	const Position& base_pos) const
{
	std::vector<Candidate> out;
	out.reserve(drones.size());

	for (std::size_t d = 0; d < drones.size(); ++d) {
		const Drone& drone = drones[d];

		const BakeryAssignment ba = find_best_bakery(drone, customer, bakeries, base_pos);
		if (ba.bakery_id < 0 || ba.achievable_amount <= 0) continue;

		const Bakery& bakery = bakeries[ba.bakery_id];

		// Defensive triple-clamp on the single-visit pickup amount:
		//   - achievable_amount : what find_best_bakery already vetted
		//   - bakery.capacity   : absolute ceiling the silo can ever hold —
		//                         blocks the "Giant-Drone" deadlock where a
		//                         drone would wait for bread the bakery is
		//                         physically incapable of producing in stock
		//   - drone free cap    : remaining payload on this drone
		// Whatever the customer still needs after this pickup will be served
		// by a subsequent round's GRASP iteration (order splitting).
		const int amount_to_take = std::min({
			ba.achievable_amount,
			bakery.capacity,
			drone.max_capacity - future_load(drone)
		});
		if (amount_to_take <= 0) continue;

		assert(amount_to_take > 0);
		assert(amount_to_take <= bakery.capacity);
		assert(amount_to_take <= bakery.current_inventory);
		assert(amount_to_take <= drone.max_capacity - future_load(drone));

		const double t_with    = compute_route_time_with_assignment(
			drone, bakery, customer, amount_to_take, base_pos);
		const double t_without = compute_route_time(drone, base_pos);
		const double dt        = std::max(t_with - t_without, kMinRouteDeltaT);

		const double ratio = static_cast<double>(amount_to_take) /
		                     static_cast<double>(std::max(customer.order_quantity, 1));
		const double score = (customer.priority_weight * ratio) / dt;

		out.push_back({static_cast<int>(d), ba.bakery_id, amount_to_take, score});
	}
	return out;
}


/* ---------- 2-Opt ---------- */

/**
 * @brief Per-customer cargo-balance validator.
 *
 * For every customer that appears in the route, the running balance of
 * (pickups - deliveries) must never go negative — the drone cannot deliver
 * bread it has not yet picked up. This is strictly stronger than a simple
 * "every delivery had a prior pickup" set check, and it is necessary because
 * a drone's route may contain multiple pickup/delivery pairs for the same
 * customer (e.g. after a partial-fulfilment re-assignment across rounds).
 *
 * Without the balance check, 2-Opt could reverse a fresh
 * [pickup_X, delivery_X] pair into [delivery_X, pickup_X] and the validator
 * would wave it through because an older committed pickup_X earlier in the
 * route already "set the flag" — yielding a phantom delivery at runtime.
 */
bool GraspSolver::is_route_valid(const std::vector<RouteNode>& route) const {
	std::map<int, int> cargo;
	for (const RouteNode& node : route) {
		if (node.customer_id < 0) continue;
		if (node.type == RouteNodeType::BAKERY_PICKUP) {
			cargo[node.customer_id] += node.bread_amount;
		} else if (node.type == RouteNodeType::CUSTOMER_DELIVERY) {
			cargo[node.customer_id] -= node.bread_amount;
			if (cargo[node.customer_id] < 0) return false;
		}
	}
	return true;
}

/**
 * @brief Classical 2-Opt over the route's *mutable suffix*.
 *
 * The mutable suffix starts after the last committed node — already-executed
 * waypoints (commit flag set in advance_drone) are anchored and never moved.
 * Each candidate reversal must (a) preserve cargo balance and (b) strictly
 * improve total distance by more than kTwoOptEpsilon.
 *
 * PRECONDITION: @p route is cargo-balance-valid on entry.
 */
void GraspSolver::apply_two_opt(std::vector<RouteNode>& route,
                                const Position& start_pos, int start_node) const
{
	if (route.size() < 2) return;
	assert(is_route_valid(route));  // upstream callers must hand us a valid plan

	std::size_t first_mutable = 0;
	for (std::size_t i = 0; i < route.size(); ++i) {
		if (route[i].committed) first_mutable = i + 1;
	}
	if (first_mutable + 1 >= route.size()) return;

	bool improved = true;
	while (improved) {
		improved = false;
		double best = path_distance(start_pos, start_node, route, 0);

		for (std::size_t i = first_mutable; i + 1 < route.size(); ++i) {
			for (std::size_t j = i + 1; j < route.size(); ++j) {
				std::vector<RouteNode> candidate = route;
				std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
				if (!is_route_valid(candidate)) continue;
				const double d = path_distance(start_pos, start_node, candidate, 0);
				if (d < best - kTwoOptEpsilon) {
					route = std::move(candidate);
					best = d;
					improved = true;
				}
			}
		}
	}
}


/* ---------- main GRASP loop ---------- */

/**
 * @brief Run one GRASP iteration end-to-end on private state snapshots.
 *
 * For each customer in priority order:
 *   - build candidates (per-drone, per-bakery, with order-splitting),
 *   - sort by score descending,
 *   - sample uniformly from the top-K = Restricted Candidate List,
 *   - extend the chosen drone's route with the pickup/delivery pair,
 *   - decrement the local bakery snapshot (so later customers see post-state),
 *   - improve the route with 2-Opt over the mutable suffix.
 *
 * The "GR" in GRASP is exactly the RCL sampling step: pure greedy would
 * always take rank 0; sampling within the top-K injects controlled
 * diversity so multiple iterations explore different solutions.
 */
GraspSolution GraspSolver::run_single_iteration(
	const std::vector<Customer>& customers,
	const std::vector<Drone>& drones,
	const std::vector<Bakery>& bakeries,
	const Position& base_pos,
	const DistanceCache& dist_cache,
	std::mt19937& rng)
{
	cache = &dist_cache;

	GraspSolution solution;
	std::vector<Drone>  local_drones   = drones;
	std::vector<Bakery> local_bakeries = bakeries;

	for (const Customer& customer : customers) {
		std::vector<Candidate> candidates =
			build_candidates(customer, local_drones, local_bakeries, base_pos);
		if (candidates.empty()) continue;

		std::sort(candidates.begin(), candidates.end(),
			[](const Candidate& a, const Candidate& b) { return a.score > b.score; });
		const int k = std::min(rcl_size, static_cast<int>(candidates.size()));
		assert(k >= 1);
		const Candidate& chosen =
			candidates[std::uniform_int_distribution<int>(0, k - 1)(rng)];

		Drone&  assigned      = local_drones[chosen.drone_idx];
		Bakery& chosen_bakery = local_bakeries[chosen.bakery_id];

		solution.intents.push_back(
			{assigned.id, chosen.bakery_id, chosen.amount, customer.id, chosen.score});

		chosen_bakery.current_inventory =
			std::max(0, chosen_bakery.current_inventory - chosen.amount);

		assigned.planned_route.push_back(
			{RouteNodeType::BAKERY_PICKUP, chosen_bakery.pos,
			 chosen_bakery.id, customer.id, chosen.amount, false});
		assigned.planned_route.push_back(
			{RouteNodeType::CUSTOMER_DELIVERY, customer.pos,
			 customer.id, customer.id, chosen.amount, false});

		apply_two_opt(assigned.planned_route,
		              assigned.current_pos, drone_start_node(assigned));
	}

	for (const Drone& d : local_drones) {
		solution.drone_routes[d.id] = d.planned_route;
	}
	return solution;
}

double GraspSolver::evaluate_solution(const std::vector<Intent>& intents) {
	double total = 0.0;
	for (const Intent& i : intents) total += i.original_score;
	return total;
}
