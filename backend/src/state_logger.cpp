#include "state_logger.hpp"
#include <fstream>
#include <iostream>

StateLogger::StateLogger(const std::string& output_path)
	: output_path(output_path) {
	rounds = nlohmann::json::array();
}

nlohmann::json StateLogger::serialize_position(const Position& pos) {
	return {{"x", pos.x}, {"y", pos.y}};
}

nlohmann::json StateLogger::serialize_bakery(const Bakery& bakery) {
	return {
		{"id", bakery.id},
		{"pos", serialize_position(bakery.pos)},
		{"current_inventory", bakery.current_inventory},
		{"capacity", bakery.capacity}
	};
}

nlohmann::json StateLogger::serialize_customer(const Customer& customer) {
	return {
		{"id", customer.id},
		{"pos", serialize_position(customer.pos)},
		{"order_quantity", customer.order_quantity},
		{"priority_weight", customer.priority_weight}
	};
}

nlohmann::json StateLogger::serialize_drone(const Drone& drone) {
	nlohmann::json route = nlohmann::json::array();
	for (const auto& node : drone.planned_route) {
		nlohmann::json jnode;
		jnode["pos"] = serialize_position(node.pos);
		jnode["entity_id"] = node.entity_id;
		jnode["bread_amount"] = node.bread_amount;
		jnode["committed"] = node.committed;
		switch (node.type) {
			case RouteNodeType::BAKERY_PICKUP:
				jnode["type"] = "pickup";
				break;
			case RouteNodeType::CUSTOMER_DELIVERY:
				jnode["type"] = "delivery";
				break;
			case RouteNodeType::BASE_RETURN:
				jnode["type"] = "base_return";
				break;
		}
		route.push_back(jnode);
	}

	return {
		{"id", drone.id},
		{"pos", serialize_position(drone.current_pos)},
		{"velocity", drone.velocity},
		{"current_load", drone.current_load},
		{"max_capacity", drone.max_capacity},
		{"is_idle", drone.is_idle},
		{"route_progress", drone.route_progress},
		{"planned_route", route}
	};
}

nlohmann::json StateLogger::serialize_intent(const Intent& intent) {
	return {
		{"drone_id", intent.drone_id},
		{"bakery_id", intent.bakery_id},
		{"requested_bread_amount", intent.requested_bread_amount},
		{"customer_id", intent.customer_id},
		{"original_score", intent.original_score}
	};
}

void StateLogger::log_round(
	int round_number,
	const std::vector<Bakery>& bakeries,
	const std::priority_queue<Customer>& customer_queue,
	const std::vector<Drone>& drones,
	const Position& base_pos,
	const std::vector<Intent>& resolved_intents
) {
	nlohmann::json round;
	round["round"] = round_number;
	round["base"] = serialize_position(base_pos);

	// Bakeries
	nlohmann::json jbakeries = nlohmann::json::array();
	for (const auto& b : bakeries) {
		jbakeries.push_back(serialize_bakery(b));
	}
	round["bakeries"] = jbakeries;

	// Customers - need to copy the queue to iterate
	nlohmann::json jcustomers = nlohmann::json::array();
	auto queue_copy = customer_queue;
	while (!queue_copy.empty()) {
		jcustomers.push_back(serialize_customer(queue_copy.top()));
		queue_copy.pop();
	}
	round["customers"] = jcustomers;

	// Drones
	nlohmann::json jdrones = nlohmann::json::array();
	for (const auto& d : drones) {
		jdrones.push_back(serialize_drone(d));
	}
	round["drones"] = jdrones;

	// Intents resolved this round
	nlohmann::json jintents = nlohmann::json::array();
	for (const auto& intent : resolved_intents) {
		jintents.push_back(serialize_intent(intent));
	}
	round["resolved_intents"] = jintents;

	rounds.push_back(round);
}

void StateLogger::flush() {
	std::ofstream file(output_path);
	if (!file.is_open()) {
		std::cerr << "Error: Could not open " << output_path << " for writing." << std::endl;
		return;
	}
	file << rounds.dump(2);
	file.close();
	std::cout << "State log written to " << output_path << std::endl;
}
