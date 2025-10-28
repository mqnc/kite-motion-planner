
#pragma once
#include "stl.h"

// function<bool(const Waypoint&, const Waypoint&)> subdivide(
// 	const function<bool(const Waypoint&, const Waypoint&)>& checkTransition,
// 	size_t numChecks
// ) {

// 	return [&checkTransition, numChecks](const Waypoint& j0, const Waypoint& j1) {
// 		for (size_t i = 0; i < numChecks; i++) {
// 			double q0 = 1.0 * i / numChecks;
// 			double q1 = 1.0 * (i + 1) / numChecks;
// 			if (!checkTransition(
// 					(1 - q0) * j0 + q0 * j1,
// 					(1 - q1) * j0 + q1 * j1
// 					)) {
// 				return false;
// 			}
// 		}
// 		return true;
// 	};
// }

Path subdivide(const Waypoint& start, const Waypoint& target, size_t numSteps) {
	Path result(numSteps + 1);
	for (size_t i = 0; i <= numSteps; i++) {
		double q = 1.0 * i / numSteps;
		result[i] = (1 - q) * start + q * target;
	}
	return result;
}

Path subdivide(const Waypoint& start, const Waypoint& target, const Waypoint& maxStepSize) {
	size_t numSteps = 1;
	for (size_t j = 0; j < start.size(); j++) {
		numSteps = max<size_t>(numSteps, ceil(abs(target[j] - start[j]) / maxStepSize[j]));
	}
	return subdivide(start, target, numSteps);
}
