#pragma once
#include "stl.h"
#include "utils/profiling.h"

bool areCollinear(const Waypoint& A, const Waypoint& B, const Waypoint& C) {

	valarray<double> AB = B - A;
	valarray<double> BC = C - B;

	double normAB = std::sqrt((AB * AB).sum());
	double normBC = std::sqrt((BC * BC).sum());

	if (normAB == 0 || normBC == 0) return true;

	AB /= normAB;
	BC /= normBC;

	for (size_t i = 0; i < AB.size(); ++i) {
		if (std::abs(AB[i] - BC[i]) >= 1e-9) return false;
	}

	return true;
}

Path skipCollinear(const Path& path) {
	if (path.size() < 3) return path;

	std::valarray<bool> skip(false, path.size());
	size_t numSkips = 0;

	for (size_t i = 0; i + 2 < path.size(); ++i) {
		if (skip[i]) { continue; }
		for (size_t j = i + 1; j + 1 < path.size(); ++j) {
			if (areCollinear(path[i], path[j], path[j + 1])) {
				skip[j] = true;
				numSkips++;
			} else {
				break;
			}
		}
	}

	Path result(path.size() - numSkips);
	size_t j = 0;
	for (size_t i = 0; i < path.size(); i++) {
		if (!skip[i]) {
			result[j++] = path[i];
		}
	}

	return result;
}


Path shortcut(
	const Path& path,
	const function<bool(const Waypoint&, const Waypoint&)>& checkTransition,
	bool moveOnAfterUnskippable = true
) {
	PROFILE_SCOPE(shortcut);

	if (path.size() <= 2) { return path; }

	size_t n = path.size();

	valarray<bool> skip(false, n);
	size_t i = 0;
	size_t numSkips = 0;

	while (i + 2 < n) {
		for (size_t j = i + 2; j < n; j++) {
			if (checkTransition(path[i], path[j])) {
				for (size_t k = i + 1; k < j; k++) {
					numSkips += !skip[k]; // increase if this was not a skip before
					skip[k] = true;
				}
			}
			else if (moveOnAfterUnskippable) {
				break;
			}
		}
		i++;
		while (skip[i]) { i++; }
	}

	Path result(n - numSkips);
	size_t j = 0;
	for (size_t i = 0; i < n; i++) {
		if (!skip[i]) {
			result[j++] = path[i];
		}
	}

	return result;
}
