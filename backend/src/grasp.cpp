#include "grasp.hpp"
#include <algorithm>
#include <limits>
#include <set>

GraspSolver::GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations)
	: graph(graph), rcl_size(rcl_size), iterations(iterations) {}

int GraspSolver::node_id_for(const RouteNode& node) const {
	switch (node.type) {
		case RouteNodeType::BAKERY_PICKUP:     return node.entity_id;
		case RouteNodeType::BASE_RETURN:       return graph.get_base_node();
		case RouteNodeType::CUSTOMER_DELIVERY: return -1; // dynamic position
	}
	return -1;
}

double GraspSolver::compute_route_time(const Drone& drone, const Position& base_pos) const {
	if (drone.planned_route.empty()) return 0.0;

	double total = 0.0;
	Position current = drone.current_pos;
	int current_node = -1;

	for (size_t i = drone.route_progress; i < drone.planned_route.size(); ++i) {
		const RouteNode& rn = drone.planned_route[i];
		int rn_node = node_id_for(rn);
		total += graph.hybrid_distance(current, current_node, rn.pos, rn_node);
		current = rn.pos;
		current_node = rn_node;
	}
	total += graph.hybrid_distance(current, current_node, base_pos, graph.get_base_node());
	return total / drone.velocity;
}

double GraspSolver::compute_route_time_with_assignment(
	const Drone& drone, const Bakery& bakery,
	const Customer& customer, const Position& base_pos) const
{
	std::vector<RouteNode> hypothetical = drone.planned_route;

	RouteNode pickup;
	pickup.type         = RouteNodeType::BAKERY_PICKUP;
	pickup.pos          = bakery.pos;
	pickup.entity_id    = bakery.id;
	pickup.customer_id  = customer.id;
	pickup.bread_amount = customer.order_quantity;
	pickup.committed    = false;

	RouteNode delivery;
	delivery.type         = RouteNodeType::CUSTOMER_DELIVERY;
	delivery.pos          = customer.pos;
	delivery.entity_id    = customer.id;
	delivery.customer_id  = customer.id;
	delivery.bread_amount = customer.order_quantity;
	delivery.committed    = false;

	hypothetical.push_back(pickup);
	hypothetical.push_back(delivery);

	double total = 0.0;
	Position current = drone.current_pos;
	int current_node = -1;
	for (size_t i = drone.route_progress; i < hypothetical.size(); ++i) {
		const RouteNode& rn = hypothetical[i];
		int rn_node = node_id_for(rn);
		total += graph.hybrid_distance(current, current_node, rn.pos, rn_node);
		current = rn.pos;
		current_node = rn_node;
	}
	total += graph.hybrid_distance(current, current_node, base_pos, graph.get_base_node());
	return total / drone.velocity;
}

// Among bakeries that can fully cover the order, pick the one with the
// shortest completion time. If none can, fall back to the highest-stock
// bakery and report only what it can actually supply.
BakeryAssignment GraspSolver::find_best_bakery(
	const Drone& drone, const Customer& customer,
	const std::vector<Bakery>& bakeries, const Position& base_pos) const
{
	BakeryAssignment best{-1, 0};
	double best_time = std::numeric_limits<double>::max();

	for (const Bakery& b : bakeries) {
		if (b.current_inventory >= customer.order_quantity) {
			double t = compute_route_time_with_assignment(drone, b, customer, base_pos);
			if (t < best_time) {
				best_time = t;
				best.bakery_id = b.id;
				best.achievable_amount = customer.order_quantity;
			}
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

double GraspSolver::route_distance(
	const std::vector<RouteNode>& route,
	const Position& start_pos, int start_node) const
{
	if (route.empty()) return 0.0;
	double dist = 0.0;
	Position current = start_pos;
	int current_node = start_node;
	for (const RouteNode& rn : route) {
		int rn_node = node_id_for(rn);
		dist += graph.hybrid_distance(current, current_node, rn.pos, rn_node);
		current = rn.pos;
		current_node = rn_node;
	}
	return dist;
}

// Per-customer pairing: a delivery for customer C must be preceded by a
// pickup tagged with the same customer_id. Stronger than counting pickups vs
// deliveries — needed because the 2-Opt result is committed verbatim into
// the drone's plan.
bool GraspSolver::is_route_valid(const std::vector<RouteNode>& route) const {
	std::set<int> picked_up;
	for (const RouteNode& node : route) {
		if (node.customer_id < 0) continue; // repositioning / base
		if (node.type == RouteNodeType::BAKERY_PICKUP) {
			picked_up.insert(node.customer_id);
		} else if (node.type == RouteNodeType::CUSTOMER_DELIVERY) {
			if (!picked_up.count(node.customer_id)) return false;
		}
	}
	return true;
}

// Standard 2-Opt over the suffix the drone hasn't visited yet. We never
// reorder nodes already executed in prior rounds (committed prefix).
void GraspSolver::apply_two_opt(
	std::vector<RouteNode>& route,
	const Position& start_pos, int start_node) const
{
	if (route.size() < 2) return;

	size_t first_mutable = 0;
	for (size_t i = 0; i < route.size(); ++i) {
		if (route[i].committed) first_mutable = i + 1;
	}
	if (first_mutable + 1 >= route.size()) return;

	bool improved = true;
	while (improved) {
		improved = false;
		double best = route_distance(route, start_pos, start_node);

		for (size_t i = first_mutable; i + 1 < route.size(); ++i) {
			for (size_t j = i + 1; j < route.size(); ++j) {
				std::vector<RouteNode> candidate = route;
				std::reverse(candidate.begin() + i, candidate.begin() + j + 1);
				if (!is_route_valid(candidate)) continue;
				double d = route_distance(candidate, start_pos, start_node);
				if (d < best - 1e-9) {
					route = std::move(candidate);
					best = d;
					improved = true;
				}
			}
		}
	}
}

// One full GRASP construction + 2-Opt local search.
//
// For each customer (in priority order), we build a candidate list of
// (drone, bakery) pairs, score each as priority * fulfillment_ratio / extra_time,
// then sample uniformly from the top-k (the RCL). We append pickup+delivery
// to the chosen drone's route and run 2-Opt to smooth the new ordering.
//
// Operates on local copies so this iteration touches no shared state.
GraspSolution GraspSolver::run_single_iteration(
	const std::vector<Customer>& customers,
	const std::vector<Drone>& drones,
	const std::vector<Bakery>& bakeries,
	const Position& base_pos,
	std::mt19937& rng)
{
	GraspSolution solution;
	std::vector<Drone>  local_drones   = drones;
	std::vector<Bakery> local_bakeries = bakeries;

	for (const Customer& customer : customers) {
		struct Candidate {
			int drone_idx;
			int bakery_id;
			int amount;
			double score;
		};
		std::vector<Candidate> candidates;

		for (size_t d = 0; d < local_drones.size(); ++d) {
			const Drone& drone = local_drones[d];

			// Future load = bread already on board + bread from pickups still
			// ahead in the route. Iterating from route_progress prevents
			// double-counting committed pickups (already in current_load).
			int future_load = drone.current_load;
			for (size_t i = drone.route_progress; i < drone.planned_route.size(); ++i) {
				if (drone.planned_route[i].type == RouteNodeType::BAKERY_PICKUP) {
					future_load += drone.planned_route[i].bread_amount;
				}
			}

			BakeryAssignment ba = find_best_bakery(drone, customer, local_bakeries, base_pos);
			if (ba.bakery_id < 0 || ba.achievable_amount <= 0) continue;
			if (future_load + ba.achievable_amount > drone.max_capacity) continue;

			const Bakery& bakery = local_bakeries[ba.bakery_id];
			double t_with    = compute_route_time_with_assignment(drone, bakery, customer, base_pos);
			double t_without = compute_route_time(drone, base_pos);
			double dt = std::max(t_with - t_without, 0.001);

			// Fulfillment ratio so partial deliveries score lower than full ones.
			double ratio = static_cast<double>(ba.achievable_amount) /
			               static_cast<double>(std::max(customer.order_quantity, 1));
			double score = (customer.priority_weight * ratio) / dt;

			candidates.push_back({static_cast<int>(d), ba.bakery_id, ba.achievable_amount, score});
		}
		if (candidates.empty()) continue;

		// RCL: rank by score DESC, sample uniformly from the top k.
		std::sort(candidates.begin(), candidates.end(),
			[](const Candidate& a, const Candidate& b) { return a.score > b.score; });
		int k = std::min(rcl_size, static_cast<int>(candidates.size()));
		Candidate& chosen = candidates[std::uniform_int_distribution<int>(0, k - 1)(rng)];

		Intent intent;
		intent.drone_id              = local_drones[chosen.drone_idx].id;
		intent.bakery_id             = chosen.bakery_id;
		intent.requested_bread_amount = chosen.amount;
		intent.customer_id           = customer.id;
		intent.original_score        = chosen.score;
		solution.intents.push_back(intent);

		Bakery& chosen_bakery = local_bakeries[chosen.bakery_id];
		chosen_bakery.current_inventory =
			std::max(0, chosen_bakery.current_inventory - chosen.amount);

		Drone& assigned = local_drones[chosen.drone_idx];

		RouteNode pickup;
		pickup.type         = RouteNodeType::BAKERY_PICKUP;
		pickup.pos          = chosen_bakery.pos;
		pickup.entity_id    = chosen_bakery.id;
		pickup.customer_id  = customer.id;
		pickup.bread_amount = chosen.amount;
		pickup.committed    = false;

		RouteNode delivery;
		delivery.type         = RouteNodeType::CUSTOMER_DELIVERY;
		delivery.pos          = customer.pos;
		delivery.entity_id    = customer.id;
		delivery.customer_id  = customer.id;
		delivery.bread_amount = chosen.amount;
		delivery.committed    = false;

		assigned.planned_route.push_back(pickup);
		assigned.planned_route.push_back(delivery);
		apply_two_opt(assigned.planned_route, assigned.current_pos);
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
