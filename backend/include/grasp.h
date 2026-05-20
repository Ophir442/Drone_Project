#pragma once

#include "types.h"
#include "distance.h"
#include <vector>
#include <random>
#include <map>

struct BakeryAssignment {
	int bakery_id;
	int achievable_amount;
};

struct GraspSolution {
	std::vector<Intent> intents;
	std::map<int, std::vector<RouteNode>> drone_routes;
};

class GraspSolver {
public:
	GraspSolver(const DeliveryGraph& graph, int rcl_size, int iterations);

	GraspSolution run_single_iteration(
		const std::vector<Customer>& customers,
		const std::vector<Drone>& drones,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos,
		std::mt19937& rng);

	static double evaluate_solution(const std::vector<Intent>& intents);

private:
	const DeliveryGraph& graph;
	int rcl_size;
	int iterations;

	double compute_route_time(const Drone& drone, const Position& base_pos) const;

	double compute_route_time_with_assignment(
		const Drone& drone,
		const Bakery& bakery,
		const Customer& customer,
		const Position& base_pos) const;

	BakeryAssignment find_best_bakery(
		const Drone& drone,
		const Customer& customer,
		const std::vector<Bakery>& bakeries,
		const Position& base_pos) const;

	void apply_two_opt(std::vector<RouteNode>& route,
	                   const Position& start_pos, int start_node = -1) const;

	bool is_route_valid(const std::vector<RouteNode>& route) const;

	double route_distance(const std::vector<RouteNode>& route,
	                      const Position& start_pos, int start_node) const;

	int node_id_for(const RouteNode& node) const;
};
