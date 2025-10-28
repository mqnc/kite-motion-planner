
#pragma once
#include "stl.h"
#include "utils/profiling.h"

Path tightenPath(
	Path path,
	const function<bool(const Waypoint&, const Waypoint&)>& checkTransition
) {
	PROFILE_SCOPE(tightenPath);

	if (path.size() < 3) { return path; }

	size_t nPoints = path.size();
	size_t nParams = path[0].size();

	auto checkTransitions = [&](size_t i, size_t j) {
		for (size_t k = i; k + 1 <= j; k++) {
			if (!checkTransition(path[k], path[k + 1])) {
				return false;
			}
		}
		return true;
	};


	auto contract = [&](double neighborWeight) {
		for (const size_t parity: {0, 1}) {
			for (size_t i = 1 + parity; i < nPoints - 1; i += 2) {
				std::cout << "tighten " << i << " (" << neighborWeight << ") ";

				auto backup = path[i];

				path[i] =
					(1.0 - neighborWeight) * path[i]
					+ 0.5 * neighborWeight * path[i - 1]
					+ 0.5 * neighborWeight * path[i + 1];

				if (!checkTransitions(i - 1, i + 1)) { // didn't work, undo and try individual parameters
					path[i] = backup;

					for (size_t k = 0; k < nParams; k++) {
						std::cout << k;

						auto backup = path[i][k];

						path[i][k] =
							(1.0 - neighborWeight) * path[i][k]
							+ 0.5 * neighborWeight * path[i - 1][k]
							+ 0.5 * neighborWeight * path[i + 1][k];

						if (!checkTransitions(i - 1, i + 1)) {
							path[i][k] = backup;
							std::cout << "X";
						}
						else { std::cout << "v"; }
					}
				}
				else { std::cout << "v"; }
				std::cout << "\n";
			}
		}
	};

	// try clamping each joint to the corridor between its start and target value
	auto clamp = [&]() {
		for (size_t k = 0; k < nParams; k++) {
			std::cout << "clamp " << k << "\n";

			auto backup = path;

			double lower = std::min(path[0][k], path[nPoints - 1][k]);
			double upper = std::max(path[0][k], path[nPoints - 1][k]);

			for (size_t i = 1; i < nPoints - 1; i++) {
				path[i][k] = std::clamp(path[i][k], lower, upper);
			}

			if (!checkTransitions(0, nPoints - 1)) {
				path = backup;
				std::cout << "X\n";
				continue;
			}
			std::cout << "v\n";
		}
	};

	clamp();
	double first = 1.0;
	double last = 0.1;
	double steps = 20;
	for (size_t iteration = 0; iteration < steps; iteration++) {
		double neighborWeight = first * pow(last / first, iteration / (steps - 1));
		contract(neighborWeight);
	}

	return path;
}
