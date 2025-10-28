
#pragma once

#include "stl.h"

Path findPath(
	const Waypoint& start,
	const std::valarray<Waypoint>& targetPool,
	function<Waypoint()> sample,
	function<bool(const Waypoint&)> checkState,
	optional<function<bool(const Waypoint&, const Waypoint&)>> checkTransition = std::nullopt, // TODO, std::function is already optional
	optional<function<double(const Waypoint&, const Waypoint&)>> distance = std::nullopt,
	const string& planner = "RRTConnect",
	const std::map<std::string, std::string>& plannerParams = {},
	const std::valarray<Waypoint>& waypointSuggestions = {},
	double jiggle = 1e-6,
	size_t maxChecks = 5000,
	double timeout = 5,
	bool verbose = false,
	function<void(const string&)> log = {}
);

struct findPathParams {
	const Waypoint start = {};
	const std::valarray<Waypoint> targetPool = {};
	function<Waypoint()> sample;
	function<bool(const Waypoint&)> checkState = [](const Waypoint&) { return false; };
	optional<function<bool(const Waypoint&, const Waypoint&)>> checkTransition = std::nullopt;
	optional<function<double(const Waypoint&, const Waypoint&)>> distance = std::nullopt;
	const string planner = "RRTConnect";
	const std::map<std::string, std::string> plannerParams = {};
	const std::valarray<Waypoint> waypointSuggestions = {};
	double jiggle = 1e-6;
	size_t maxChecks = 5000;
	double timeout = 5;
	bool verbose = false;
	function<void(const string&)> log = {};
};

inline Path findPath(findPathParams params){
	return findPath(
		params.start,
		params.targetPool,
		params.sample,
		params.checkState,
		params.checkTransition,
		params.distance,
		params.planner,
		params.plannerParams,
		params.waypointSuggestions,
		params.jiggle,
		params.maxChecks,
		params.timeout,
		params.verbose,
		params.log
	);
}