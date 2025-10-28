#pragma once

#include "stl.h"
#include "transform.h"
#include "json.hpp"

void to_json(json& j, const Vector3& v) { j = json {v[0], v[1], v[2]}; }
void from_json(const json& j, Vector3& v) { v = Vector3 {j[0], j[1], j[2]}; }

struct Check {
	vector<Vector3> points;
	bool ok;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE(Check, points, ok);
};

using Trace = vector<variant<Check, string>>;

void to_json(json& j, const Trace& t) {
	j = json::array();
	for (const auto& item: t) {
		std::visit([&](auto&& arg) { j.push_back(arg); }, item);
	}
}

class Tracer {
	vector<Trace> traces;
public:
	Tracer() { newTrace(); }
	void append(const Check& check) { traces.back().push_back(check); }
	void append(const string& msg) { traces.back().push_back(msg); }
	void newTrace() { traces.push_back({}); }
	friend void to_json(json& j, const Tracer& t) {
		j = json::array();
		for (const auto& trace: t.traces) {
			j.push_back(trace);
		}
	}
};
