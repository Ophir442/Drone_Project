#pragma once

#include "types.h"
#include <unordered_map>
#include <vector>

/**
 * @brief Static distance graph over bakeries + base.
 *
 * Built once during Simulation::initialize and never mutated afterwards.
 * The base is always the last node so its index is @c node_count - 1.
 *
 * Storage is a flat row-major vector<double>: one allocation, contiguous,
 * L1-friendly for sequential row sweeps (the hot path in path_distance).
 */
class DeliveryGraph {
public:
	/// Populate the symmetric adjacency matrix. Base is appended last.
	void initialize(const std::vector<Bakery>& bakeries, const Position& base_pos);

	/// O(1) lookup; both endpoints must be valid node ids in [0, node_count).
	double get_static_distance(int from_node, int to_node) const;

	/// Plain Euclidean — exposed for callers without node ids.
	static double compute_distance(const Position& a, const Position& b);

	/// Matrix lookup when both nodes are in range; Euclidean fallback otherwise.
	double hybrid_distance(const Position& a, int a_node,
	                       const Position& b, int b_node) const;

	int get_node_count() const noexcept { return node_count; }
	int get_base_node()  const noexcept { return node_count - 1; }

private:
	std::vector<double> adjacency_matrix;   ///< row-major, size n*n
	int node_count = 0;
};

/**
 * @brief Per-round, broader distance cache.
 *
 * Covers every entity GRASP touches in a single round: all bakeries, every
 * active customer, every drone's current position, and the base. Built once
 * on the main thread before parallel GRASP dispatches, so worker threads
 * never recompute Euclidean distances inside the iteration loop.
 *
 * Single contiguous matrix instead of vector<vector<double>> — eliminates the
 * pointer-chase per row and keeps the entire table inside L1 for typical
 * round sizes (~30 entities × 8 B = 7.2 KB).
 */
class DistanceCache {
public:
	/// Rebuild the matrix and id->index maps from the given round snapshot.
	void build(const std::vector<Bakery>& bakeries,
	           const std::vector<Customer>& customers,
	           const std::vector<Drone>& drones,
	           const Position& base_pos);

	/// O(1) matrix lookup; caller must pass valid node indices.
	double dist(int a, int b) const noexcept {
		return matrix[static_cast<std::size_t>(a) * node_count + b];
	}

	int bakery_node(int bakery_id) const;
	int customer_node(int customer_id) const;
	int drone_node(int drone_id) const;
	int base_node() const noexcept { return base_idx; }

	/// Matrix-first lookup with Euclidean fallback for unknown node ids.
	double lookup(const Position& a, int node_a,
	              const Position& b, int node_b) const;

private:
	std::vector<double>          matrix;    ///< row-major, size n*n
	std::unordered_map<int, int> bakery_to_idx;
	std::unordered_map<int, int> customer_to_idx;
	std::unordered_map<int, int> drone_to_idx;
	int base_idx   = -1;
	int node_count = 0;
};
