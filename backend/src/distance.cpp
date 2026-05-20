#include "distance.h"

namespace {

/// Fill a symmetric Euclidean distance matrix from a list of positions.
void fill_symmetric_matrix(const std::vector<Position>& positions,
                           std::vector<std::vector<double>>& matrix) {
	const int n = static_cast<int>(positions.size());
	matrix.assign(n, std::vector<double>(n, 0.0));
	for (int i = 0; i < n; ++i) {
		for (int j = i + 1; j < n; ++j) {
			const double d = positions[i].distance_to(positions[j]);
			matrix[i][j] = d;
			matrix[j][i] = d;
		}
	}
}

}  // namespace


/* ---------- DeliveryGraph ---------- */

void DeliveryGraph::initialize(const std::vector<Bakery>& bakeries,
                               const Position& base_pos) {
	std::vector<Position> positions;
	positions.reserve(bakeries.size() + 1);
	for (const auto& b : bakeries) positions.push_back(b.pos);
	positions.push_back(base_pos);  // base is always the last node

	fill_symmetric_matrix(positions, adjacency_matrix);
	node_count = static_cast<int>(positions.size());
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


/* ---------- DistanceCache ---------- */

void DistanceCache::build(const std::vector<Bakery>& bakeries,
                          const std::vector<Customer>& customers,
                          const std::vector<Drone>& drones,
                          const Position& base_pos) {
	bakery_to_idx.clear();
	customer_to_idx.clear();
	drone_to_idx.clear();

	std::vector<Position> positions;
	positions.reserve(bakeries.size() + customers.size() + drones.size() + 1);

	// Generic registration: assign the next free index to an entity and stash its position.
	auto register_entity = [&positions](auto& map, int id, const Position& pos) {
		map[id] = static_cast<int>(positions.size());
		positions.push_back(pos);
	};

	for (const auto& b : bakeries)  register_entity(bakery_to_idx,   b.id, b.pos);
	for (const auto& c : customers) register_entity(customer_to_idx, c.id, c.pos);
	for (const auto& d : drones)    register_entity(drone_to_idx,    d.id, d.current_pos);
	base_idx = static_cast<int>(positions.size());
	positions.push_back(base_pos);

	fill_symmetric_matrix(positions, matrix);
	node_count = static_cast<int>(positions.size());
}

int DistanceCache::bakery_node(int bakery_id) const {
	const auto it = bakery_to_idx.find(bakery_id);
	return it == bakery_to_idx.end() ? -1 : it->second;
}

int DistanceCache::customer_node(int customer_id) const {
	const auto it = customer_to_idx.find(customer_id);
	return it == customer_to_idx.end() ? -1 : it->second;
}

int DistanceCache::drone_node(int drone_id) const {
	const auto it = drone_to_idx.find(drone_id);
	return it == drone_to_idx.end() ? -1 : it->second;
}

double DistanceCache::lookup(const Position& a, int node_a,
                             const Position& b, int node_b) const {
	if (node_a >= 0 && node_b >= 0 && node_a < node_count && node_b < node_count) {
		return matrix[node_a][node_b];
	}
	return a.distance_to(b);
}
