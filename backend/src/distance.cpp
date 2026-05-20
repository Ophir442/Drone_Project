#include "distance.h"

using namespace std;

void DeliveryGraph::initialize(const vector<Bakery>& bakeries, const Position& base_pos) {
	node_count = static_cast<int>(bakeries.size()) + 1;
	adjacency_matrix.assign(node_count, vector<double>(node_count, 0.0));

	vector<Position> positions;
	positions.reserve(node_count);
	for (const auto& b : bakeries) positions.push_back(b.pos);
	positions.push_back(base_pos); // base is the last node

	for (int i = 0; i < node_count; ++i) {
		for (int j = i + 1; j < node_count; ++j) {
			double d = positions[i].distance_to(positions[j]);
			adjacency_matrix[i][j] = d;
			adjacency_matrix[j][i] = d;
		}
	}
}

double DeliveryGraph::get_static_distance(int from_node, int to_node) const {
	return adjacency_matrix[from_node][to_node];
}

double DeliveryGraph::compute_distance(const Position& a, const Position& b) {
	return a.distance_to(b);
}

double DeliveryGraph::hybrid_distance(const Position& a, int a_node,
                                      const Position& b, int b_node) const {
	if (a_node >= 0 && b_node >= 0 && a_node < node_count && b_node < node_count) {
		return adjacency_matrix[a_node][b_node];
	}
	return a.distance_to(b);
}
