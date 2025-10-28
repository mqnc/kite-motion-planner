#pragma once
#include "stl.h"
#include "utils/profiling.h"

bool verifyPath(
	const Path& path,
	const function<bool(const Waypoint&, const Waypoint&)>& checkTransition
) {
	PROFILE_SCOPE(verifyPath);

	if (path.size() == 0) { throw runtime_error("trying to verify empty path"); }
	if (path.size() == 1) { return checkTransition(path[0], path[0]); }

	for (size_t i = 0; i + 1 < path.size(); i++) {
		if (!checkTransition(path[i], path[i + 1])) {
			return false;
		}
	}

	return true;
}
