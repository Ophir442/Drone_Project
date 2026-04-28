#pragma once

#include "types.hpp"
#include <vector>

class DeliveryGraph {
public:
	// Initialize adjacency matrix for bakeries + base
	// Node indices: 0..num_bakeries-1 = bakeries, num_bakeries = base
	void initialize(const std::vector<Bakery>& bakeries, const Position& base_pos);

	// Get precomputed distance between two static nodes (bakeries/base)
	double get_static_distance(int from_node, int to_node) const;

	// On-the-fly Euclidean distance to/from a customer position
	static double compute_distance(const Position& a, const Position& b);

	// Hybrid distance lookup:
	//   If both endpoints are static nodes (bakery or base), O(1) matrix fetch.
	//   Otherwise, falls back to Euclidean.
	//   Pass node_id >= 0 for bakeries, base_node_id for the base, or -1 for dynamic positions.
	double hybrid_distance(const Position& a, int a_node, const Position& b, int b_node) const;

	int get_node_count() const { return node_count; }
	int get_base_node() const { return node_count - 1; }

private:
	std::vector<std::vector<double>> adjacency_matrix;
	int node_count = 0;
};
