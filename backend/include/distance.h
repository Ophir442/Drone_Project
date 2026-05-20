#pragma once

#include "types.h"
#include <vector>

class DeliveryGraph {
public:
	void initialize(const std::vector<Bakery>& bakeries, const Position& base_pos);

	double get_static_distance(int from_node, int to_node) const;

	static double compute_distance(const Position& a, const Position& b);

	// O(1) matrix lookup if both endpoints are static (bakery or base);
	// otherwise falls back to Euclidean. Pass -1 for dynamic positions.
	double hybrid_distance(const Position& a, int a_node, const Position& b, int b_node) const;

	int get_node_count() const { return node_count; }
	int get_base_node() const { return node_count - 1; }

private:
	std::vector<std::vector<double>> adjacency_matrix;
	int node_count = 0;
};
