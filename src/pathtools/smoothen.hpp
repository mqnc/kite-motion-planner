
#pragma once

#include "stl.h"
#include "interpolator/interpolator.h"
#include "valarraytools.h"
#include "subspace.hpp"
#include "timeparametrize.hpp"
#include "utils/profiling.h"

// generate velocity vectors in waypoints using heuristics similar to akima spline
// then sample to find a good scaling for each of those vectors

Path smoothenPath(
	Path waypoints,
	double dt,
	const valarray<double>& maxVelocity,
	const valarray<double>& maxAcceleration,
	const valarray<double>& maxJerk,
    const function<bool(const Path&)>& checkPath,
    const Subspace* subspace = nullptr
) {
    PROFILE_SCOPE(smoothenPath);

	// https://en.wikipedia.org/wiki/Off-by-one_error#Fencepost_error
	// waypoints are fence posts, spline segments are fence sections

	// eliminate duplicates
	valarray<bool> uniqueMask(true, waypoints.size());
	for (size_t wp = 0; wp + 1 < waypoints.size(); wp++) {
		if (abs(waypoints[wp] - waypoints[wp + 1]).max() < 1e-12) {
			uniqueMask[wp + 1] = false;
		}
	}
	waypoints = waypoints[uniqueMask];

	if (waypoints.size() <= 1) { return waypoints; }

	size_t numWaypoints = waypoints.size();
	size_t numSegments = numWaypoints - 1;
	size_t numDimensions = waypoints[0].size();

	// vectors between consecutive waypoints
	valarray<valarray<double>> betweens = diff(waypoints);

	// set distance measure for segment equal to maximum component difference
	// (can't be 0, we eliminated duplicate waypoints)
	valarray<double> distanceMeasure(betweens.size());
	for (size_t i = 0; i < betweens.size(); i++) {
		distanceMeasure[i] = abs(betweens[i]).max();
	}

	// what is the fastest the robot could travel on linear segments
	valarray<valarray<double>> maxLinearSegmentVelocities = betweens;
	for (auto& v : maxLinearSegmentVelocities) {
		v = maximizeLengthWithinCuboid(v, maxVelocity);
	}

	// where two segments meet in a waypoint, how heavily does each segment's
	// velocity contribute to the velocity in the waypoint
	valarray<double> velContribution = 1.0 / distanceMeasure;

    valarray<double> zeros = valarray<double>(0.0, numDimensions);

	valarray<valarray<double>> maxWaypointVelocities(numWaypoints);
	maxWaypointVelocities[0] = zeros;
	maxWaypointVelocities[maxWaypointVelocities.size() - 1] = zeros;
	for (size_t wp = 1; wp < numWaypoints - 1; wp++) {
		maxWaypointVelocities[wp] = (
			maxLinearSegmentVelocities[wp - 1] * velContribution[wp - 1]
			+ maxLinearSegmentVelocities[wp] * velContribution[wp]
			) / (velContribution[wp - 1] + velContribution[wp]);
	}

	valarray<double> waypointSpeedScale(0.0, numWaypoints);

	auto trajectorize = [&](size_t seg)->unique_ptr<ContinuousTrajectory> {

		if (waypointSpeedScale[seg] == 0 && waypointSpeedScale[seg + 1] == 0) {
			return make_unique<LinearPointToPointTrajectory>(
				waypoints[seg],
				waypoints[seg + 1],
				maxVelocity,
				maxAcceleration,
				maxJerk
				);
		}
		else {
			valarray<double> vStart = maxWaypointVelocities[seg] * waypointSpeedScale[seg];
			valarray<double> vEnd = maxWaypointVelocities[seg + 1] * waypointSpeedScale[seg + 1];

			return make_unique<OptimalTrajectory>(
				waypoints[seg], vStart, zeros,
				waypoints[seg + 1], vEnd, zeros,
				maxVelocity,
				maxAcceleration,
				maxJerk
				);
		}
	};

	auto cost = [&](size_t seg) -> double {

		auto testTrack = trajectorize(seg);
		double work = 0;

		valarray<valarray<double>> sampledTestTrack = testTrack->sample(dt, false);

		// force into constraints
        if (subspace){
		    sampledTestTrack = subspace->unproject(subspace->project(sampledTestTrack, false, inf));
        }

		bool clear = checkPath(sampledTestTrack);
		if (!clear) { return inf; }

		// cost represents integral over absolute value of acceleration
		// in order to minimize time and oscillations
		auto acc = diff(diff(sampledTestTrack));
		for (const auto& a : acc) {
			work += abs(a).sum();
		}
		return work;
	};

	double granularity = 0.499;
	for (size_t run = 0; run < 3; run++) {
		for (size_t wp = 1; wp < numWaypoints - 1; wp++) {
			double best = 0;
			double lowestCost = inf;
			for (double factor = 0; factor < 1; factor += granularity) {
				waypointSpeedScale[wp] = factor;
				double testCost1 = cost(wp - 1);
				if (std::isinf(testCost1)) { continue; }
				double testCost2 = cost(wp);
				if (std::isinf(testCost2)) { continue; }
				double testCost = testCost1 + testCost2;
				if (testCost < lowestCost) {
					best = factor;
					lowestCost = testCost;
				}
			}
			waypointSpeedScale[wp] = best;
		}
		granularity /= 2;
	}

	CompoundTrajectory compound;
	for (size_t i = 0; i < numSegments; i++) {
		compound.segments.push_back(trajectorize(i));
	}

	auto result = compound.sample(dt, true);

	// force into constraints
    if (subspace){
    	result = subspace->unproject(subspace->project(result, false, inf));
    }

	// remove overshoots from constraint enforcement
	result = timeParameterize(
		result, dt,
		maxVelocity,
		maxAcceleration,
		maxJerk
	);

	// plot(result, "exports/x.py");
	// plot(diff(result, dt), "exports/v.py");
	// plot(diff(diff(result, dt), dt), "exports/a.py");
	// plot(diff(diff(diff(result, dt), dt), dt), "exports/j.py");
	return result;
}
