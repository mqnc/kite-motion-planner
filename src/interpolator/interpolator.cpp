

#include "interpolator.h"
#include "ruckig/ruckig.hpp"
#include "valarraytools.h"

std::unordered_map<ruckig::Result, string> ruckigStatusCodes {
	{ruckig::Result::Working,
		"The trajectory is calculated normally"},
	{ruckig::Result::Finished,
		"Trajectory has reached its final position"},
	{ruckig::Result::Error,
		"Unclassified error"},
	{ruckig::Result::ErrorInvalidInput,
		"Error in the input parameter"},
	{ruckig::Result::ErrorTrajectoryDuration,
		"The trajectory duration exceeds its numerical limits"},
	{ruckig::Result::ErrorPositionalLimits,
		"The trajectory exceeds the given positional limits (only in Ruckig Pro)"},
	{ruckig::Result::ErrorExecutionTimeCalculation,
		"Error during the extremel time calculation (Step 1)"},
	{ruckig::Result::ErrorSynchronizationCalculation,
		"Error during the synchronization calculation (Step 2)"}
	//{ruckig::Result::ErrorNoPhaseSynchronization,
	//	"The trajectory cannot be phase synchronized"}
};

void checkRuckigResult(ruckig::Result r) {
	if (r < 0) {
		if (ruckigStatusCodes.count(r)) {
			throw runtime_error(string()
				+ "interpolation error: "
				+ ruckigStatusCodes[r]);
		}
		else {
			throw runtime_error("unknown interpolation error");
		}
	}
}

valarray<valarray<double>> ContinuousTrajectory::sample(
	double dt, bool forceSampleAtTarget) const {

	size_t n = floor(getDuration() / dt) + 1 + forceSampleAtTarget;
	std::valarray<std::valarray<double>> result(n);
	for (size_t k = 0; k < n; k++) {
		result[k] = at(std::min(k * dt, getDuration()));
	}
	return result;
}

valarray<valarray<double>> ContinuousTrajectory::sampleByDistance(
	double dtTest, double dDistanceApprox) const {

	vector<valarray<double>> buffer;

	double t = 0;
	valarray<double> pLast = at(0);
	buffer.push_back(pLast);
	bool done = false;
	while (!done) {
		double tTestLast = t;
		double lastOvershoot = 0;
		while (true) {
			double tTest = tTestLast + dtTest;
			if (tTest >= getDuration()) {
				t = getDuration();
				done = true;
				break;
			}
			auto pTest = at(tTest);
			valarray<double> delta = pTest - pLast;
			double d = norm(delta);
			double overshoot = d / dDistanceApprox;
			if (overshoot > 1) {
				double fraction = (1 - lastOvershoot) / (overshoot - lastOvershoot);
				t = tTestLast + fraction * dtTest;
				break;
			}
			tTestLast = tTest;
			lastOvershoot = overshoot;
		}
		auto p = at(t);
		buffer.push_back(p);
		pLast = p;
	}

	valarray<valarray<double>> result (buffer.data(), buffer.size());
	return result;
}

LinearPointToPointTrajectory::LinearPointToPointTrajectory(
	const valarray<double>& p0,
	const valarray<double>& p1,
	const valarray<double>& maxVelocity,
	const valarray<double>& maxAcceleration,
	const valarray<double>& maxJerk
	): p0 {p0}, p1 {p1} {
	size_t dof = p0.size();

	ruckig::Ruckig<1> interpolator;
	ruckig::InputParameter<1> input;

	valarray<double> delta = p1 - p0;
	double length = norm(delta);

	if (length < 1e-15) {
		input.max_velocity[0] = 1;
		input.max_acceleration[0] = 1;
		input.max_jerk[0] = 1;
		input.synchronization = ruckig::Synchronization::Time;

		ruckig::Result status = interpolator.calculate(input, progress);
		checkRuckigResult(status);
		return;
	}

	valarray<double> direction = delta / length;

	input.current_position[0] = 0;
	input.current_velocity[0] = 0;
	input.current_acceleration[0] = 0;
	input.target_position[0] = 1;
	input.target_velocity[0] = 0;
	input.target_acceleration[0] = 0;

	double projectedMaxVelocity = (maxVelocity / abs(direction)).min();
	double projectedMaxAcceleration = (maxAcceleration / abs(direction)).min();
	double projectedMaxJerk = (maxJerk / abs(direction)).min();

	input.max_velocity[0] = projectedMaxVelocity / length;
	input.max_acceleration[0] = projectedMaxAcceleration / length;
	input.max_jerk[0] = projectedMaxJerk / length;
	input.synchronization = ruckig::Synchronization::Time;

	ruckig::Result status = interpolator.calculate(input, progress);
	checkRuckigResult(status);
}

double LinearPointToPointTrajectory::getDuration() const {
	return progress.get_duration();
}

valarray<double> LinearPointToPointTrajectory::at(double t) const {
	if (t < 0) { t = 0; }
	if (t > getDuration()) { t = getDuration(); }
	array<double, 1> pos, v, a;
	progress.at_time(t, pos, v, a);
	return (1.0 - pos[0]) * p0 + pos[0] * p1;
}



OptimalTrajectory::OptimalTrajectory(
	const valarray<double>& startPosition,
	const valarray<double>& startVelocity,
	const valarray<double>& startAcceleration,
	const valarray<double>& targetPosition,
	const valarray<double>& targetVelocity,
	const valarray<double>& targetAcceleration,
	const valarray<double>& maxVelocity,
	const valarray<double>& maxAcceleration,
	const valarray<double>& maxJerk
	): trajectory {startPosition.size()} {

	size_t dof = startPosition.size();

	ruckig::Ruckig<ruckig::DynamicDOFs> interpolator {dof};
	ruckig::InputParameter<ruckig::DynamicDOFs> input {dof};
	ruckig::OutputParameter<ruckig::DynamicDOFs> output {dof};

	input.current_position =
		vector<double>(begin(startPosition), end(startPosition));
	input.current_velocity =
		vector<double>(begin(startVelocity), end(startVelocity));
	input.current_acceleration =
		vector<double>(begin(startAcceleration), end(startAcceleration));
	input.target_position =
		vector<double>(begin(targetPosition), end(targetPosition));
	input.target_velocity =
		vector<double>(begin(targetVelocity), end(targetVelocity));
	input.target_acceleration =
		vector<double>(begin(targetAcceleration), end(targetAcceleration));
	input.max_velocity =
		vector<double>(begin(maxVelocity), end(maxVelocity));
	input.max_acceleration =
		vector<double>(begin(maxAcceleration), end(maxAcceleration));
	input.max_jerk =
		vector<double>(begin(maxJerk), end(maxJerk));
	input.synchronization = ruckig::Synchronization::Time;
	// input.duration_discretization = ruckig::DurationDiscretization::Discrete;

	ruckig::Result status = interpolator.calculate(input, trajectory);

	checkRuckigResult(status);
}

double OptimalTrajectory::getDuration() const {
	return trajectory.get_duration();
}

valarray<double> OptimalTrajectory::at(double t) const {
	const size_t dof = trajectory.degrees_of_freedom;
	vector<double> pos(dof), v(dof), a(dof);
	trajectory.at_time(t, pos, v, a);
	return valarray<double>(pos.data(), pos.size());
}


// QuinticSplineTrajectory::QuinticSplineTrajectory(
// 	const valarray<double>& startPosition,
// 	const valarray<double>& startVelocity,
// 	const valarray<double>& startAcceleration,
// 	const valarray<double>& targetPosition,
// 	const valarray<double>& targetVelocity,
// 	const valarray<double>& targetAcceleration,
// 	double duration
// 	):
// 	spline { 0, duration,
// 		startPosition,
// 		startVelocity,
// 		startAcceleration,
// 		targetPosition,
// 		targetVelocity,
// 		targetAcceleration
// 	},
// 	duration {duration}
// {}

// double QuinticSplineTrajectory::getDuration() const {
// 	return duration;
// }

// valarray<double> QuinticSplineTrajectory::at(double t) const {
// 	return spline(t);
// }



double CompoundTrajectory::getDuration() const {
	double sum = 0;
	for (const auto& s: segments) {
		sum += s->getDuration();
	}
	return sum;
}

valarray<double> CompoundTrajectory::at(double t) const {
	double offset = 0;
	for (size_t i = 0; i < segments.size(); i++) {
		const double endOfSegment = offset + segments[i]->getDuration();
		if (t < endOfSegment) {
			return segments[i]->at(t - offset);
		}
		offset = endOfSegment;
	}
	return segments.back()->at(t);
}

valarray<valarray<double>> CompoundTrajectory::sample(
	double dt,
	bool forceSampleAtTarget
) const {

	size_t n = floor(getDuration() / dt) + 1 + forceSampleAtTarget;
	std::valarray<std::valarray<double>> result(n);
	size_t s = 0;
	size_t k = 0;
	double offset = 0;
	while (s < segments.size()) {
		for (; k * dt <= offset + segments[s]->getDuration(); k++) {
			result[k] = segments[s]->at(k * dt - offset);
		}
		offset += segments[s]->getDuration();
		s++;
	}
	for (; k < n; k++) {
		result[k] = segments[s - 1]->at(segments[s - 1]->getDuration());
	}
	return result;
}
