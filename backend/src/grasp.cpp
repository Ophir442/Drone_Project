#include "grasp.h"
#include <algorithm>
#include <limits>
#include <set>

namespace {

constexpr double kMinRouteDeltaT = 0.001;  ///< floor for dt in score division
constexpr double kTwoOptEpsilon  = 1e-9;   ///< improvement threshold

/// Sum of bread already committed on this drone (current load + scheduled pickups).
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

/// Single dispatch point for "look up the distance between two waypoints".
/// Uses the per-round matrix when available, falls back to the static
/// graph (which itself falls back to Euclidean) otherwise.
double GraspSolver::distance_between(const Position& a, int a_node,
                                     const Position& b, int b_node) const {
	return cache ? cache->lookup(a, a_node, b, b_node)
	             : graph.hybrid_distance(a, a_node, b, b_node);
}

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

/// Walk a route slice from `start_pos` and sum hop distances. When
/// `end_pos` is non-null the final hop to that endpoint is added too.
/// Used by every distance-summation in this file (DRY hub).
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

double GraspSolver::compute_route_time(const Drone& drone,
                                       const Position& base_pos) const {
	if (drone.planned_route.empty()) return 0.0;
	const double d = path_distance(
		drone.current_pos, drone_start_node(drone),
		drone.planned_route, drone.route_progress,
		&base_pos, base_node_id());
	return d / drone.velocity;
}

double GraspSolver::compute_route_time_with_assignment(
	const Drone& drone, const Bakery& bakery, const Customer& customer,
	int amount, const Position& base_pos) const
{
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

/// Prefer the bakery that can fully cover the order with the shortest extra
/// time. If none can, fall back to the highest-stock bakery and accept a
/// partial achievable amount.
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
	return best;
}

/// Enumerate every (drone, bakery, amount) that could serve this customer,
/// scored by  (priority * fulfilment_ratio) / Δroute_time. Order splitting
/// keeps a candidate alive whenever the drone has *any* free capacity, even
/// if the bakery could supply more.
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

		// Order splitting: take whatever fits in remaining capacity rather
		// than rejecting the drone for not covering the whole order.
		const int amount_to_take = std::min(
			ba.achievable_amount, drone.max_capacity - future_load(drone));
		if (amount_to_take <= 0) continue;

		const Bakery& bakery = bakeries[ba.bakery_id];
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

/// A delivery for customer C must be preceded by a pickup tagged with the same
/// customer_id. Required because 2-Opt may reverse a segment that crosses a
/// pickup→delivery pair, which would otherwise yield an unservable route.
bool GraspSolver::is_route_valid(const std::vector<RouteNode>& route) const {
	std::set<int> picked_up;
	for (const RouteNode& node : route) {
		if (node.customer_id < 0) continue;
		if (node.type == RouteNodeType::BAKERY_PICKUP) {
			picked_up.insert(node.customer_id);
		} else if (node.type == RouteNodeType::CUSTOMER_DELIVERY) {
			if (!picked_up.count(node.customer_id)) return false;
		}
	}
	return true;
}

/// 2-Opt over the mutable suffix only (the committed prefix is left alone).
/// Reverses [i, j+1] while route validity holds and total distance strictly
/// improves by more than kTwoOptEpsilon.
void GraspSolver::apply_two_opt(std::vector<RouteNode>& route,
                                const Position& start_pos, int start_node) const
{
	if (route.size() < 2) return;

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

		// RCL: rank by score DESC, sample uniformly from the top k. The
		// greedy bias plus the random tail is the "GR" in GRASP — multiple
		// iterations explore different paths through the assignment space.
		std::sort(candidates.begin(), candidates.end(),
			[](const Candidate& a, const Candidate& b) { return a.score > b.score; });
		const int k = std::min(rcl_size, static_cast<int>(candidates.size()));
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
