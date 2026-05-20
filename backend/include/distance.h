#pragma once

#include "types.h"
#include <vector>
#include <unordered_map>

/// Static distance graph: pre-computed bakery↔bakery and bakery↔base matrix.
/// Built once during simulation initialization and never mutated afterwards.
class DeliveryGraph {
public:
	/// Populate the symmetric adjacency matrix. Base is the last node.
	void initialize(const std::vector<Bakery>& bakeries, const Position& base_pos);

	/// O(1) lookup; both endpoints must be valid node ids.
	double get_static_distance(int from_node, int to_node) const;

	/// Plain Euclidean — exposed for callers that don't have node ids.
	static double compute_distance(const Position& a, const Position& b);

	/// Matrix lookup when both nodes are in range; Euclidean fallback otherwise.
	double hybrid_distance(const Position& a, int a_node,
	                       const Position& b, int b_node) const;

	int get_node_count() const noexcept { return node_count; }
	int get_base_node()  const noexcept { return node_count - 1; }

private:
	std::vector<std::vector<double>> adjacency_matrix;
	int node_count = 0;
};

/// Per-round distance cache: every entity GRASP touches (bakeries, the active
/// customer set, drone start positions, base). Built once on the main thread
/// before parallel GRASP tasks dispatch, so worker threads never recompute
/// Euclidean distances inside the iteration loop.
class DistanceCache {
public:
	/// Rebuild the matrix for the given round snapshot.
	void build(const std::vector<Bakery>& bakeries,
	           const std::vector<Customer>& customers,
	           const std::vector<Drone>& drones,
	           const Position& base_pos);

	/// O(1) matrix lookup (caller must pass valid node indices).
	double dist(int a, int b) const noexcept { return matrix[a][b]; }

	int bakery_node(int bakery_id) const;
	int customer_node(int customer_id) const;
	int drone_node(int drone_id) const;
	int base_node() const noexcept { return base_idx; }

	/// Matrix-first lookup with Euclidean fallback for unknown node ids.
	double lookup(const Position& a, int node_a,
	              const Position& b, int node_b) const;

private:
	std::vector<std::vector<double>> matrix;
	std::unordered_map<int, int> bakery_to_idx;
	std::unordered_map<int, int> customer_to_idx;
	std::unordered_map<int, int> drone_to_idx;
	int base_idx = -1;
	int node_count = 0;
};
