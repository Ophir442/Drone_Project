#pragma once

#include "types.hpp"
#include <nlohmann/json.hpp>
#include <vector>
#include <queue>
#include <string>

class StateLogger {
public:
	explicit StateLogger(const std::string& output_path);

	// Log the state of one round
	void log_round(
		int round_number,
		const std::vector<Bakery>& bakeries,
		const std::priority_queue<Customer>& customer_queue,
		const std::vector<Drone>& drones,
		const Position& base_pos,
		const std::vector<Intent>& resolved_intents
	);

	// Write all logged rounds to file
	void flush();

private:
	std::string output_path;
	nlohmann::json rounds;

	nlohmann::json serialize_position(const Position& pos);
	nlohmann::json serialize_bakery(const Bakery& bakery);
	nlohmann::json serialize_customer(const Customer& customer);
	nlohmann::json serialize_drone(const Drone& drone);
	nlohmann::json serialize_intent(const Intent& intent);
};
