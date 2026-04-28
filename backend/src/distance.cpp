#include "distance.hpp"

void DeliveryGraph::initialize(const std::vector<Bakery>& bakeries, const Position& base_pos) {
	// Nodes: bakeries[0..n-1] + base[n]
	node_count = static_cast<int>(bakeries.size()) + 1;
	adjacency_matrix.resize(node_count, std::vector<double>(node_count, 0.0));

	// Collect all static positions
	std::vector<Position> positions;
	for (const auto& b : bakeries) {
		positions.push_back(b.pos);
	}
	positions.push_back(base_pos); // base is the last node

	// Fill the adjacency matrix with Euclidean distances
	for (int i = 0; i < node_count; ++i) {
		for (int j = i + 1; j < node_count; ++j) {
			double dist = positions[i].distance_to(positions[j]);
			adjacency_matrix[i][j] = dist;
			adjacency_matrix[j][i] = dist;
		}
	}
}

double DeliveryGraph::get_static_distance(int from_node, int to_node) const {
	return adjacency_matrix[from_node][to_node];
}

double DeliveryGraph::compute_distance(const Position& a, const Position& b) {
	return a.distance_to(b);
}

double DeliveryGraph::hybrid_distance(const Position& a, int a_node, const Position& b, int b_node) const {
	// Both endpoints are static nodes — O(1) matrix lookup
	if (a_node >= 0 && b_node >= 0 && a_node < node_count && b_node < node_count) {
		return adjacency_matrix[a_node][b_node];
	}
	// At least one endpoint is dynamic — compute Euclidean
	return a.distance_to(b);
}
