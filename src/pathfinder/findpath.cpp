
#include "findpath.h"
#include "planners.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalStates.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/samplers/DeterministicStateSampler.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/samplers/deterministic/HaltonSequence.h>
#include <boost/math/special_functions/prime.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;



auto arrayFromState(const ob::State* state, size_t n) {
	const auto* rvState = state->as<ob::RealVectorStateSpace::StateType>();
	Waypoint buffer(n);

	for (size_t i = 0; i < buffer.size(); i++) {
		buffer[i] = rvState->values[i];
	}
	return buffer;
};


class ModdedRealVectorStateSpace: public ob::RealVectorStateSpace {
	function<double(
		const Waypoint&,
		const Waypoint&
		)> distanceCallback;
public:
	ModdedRealVectorStateSpace(
		unsigned int dim,
		function<double(
			const Waypoint&,
			const Waypoint&
			)> distfn
		):
		RealVectorStateSpace(dim),
		distanceCallback {distfn} {}

	double distance(const ob::State* state1, const ob::State* state2) const override {
		const Waypoint wp1(state1->as<StateType>()->values, getDimension());
		const Waypoint wp2(state2->as<StateType>()->values, getDimension());
		return distanceCallback(wp1, wp2);
	}
};


class CustomSampler: public ob::StateSampler {
	const function<Waypoint()> sample;
	size_t numDims;

public:
	CustomSampler(const ob::StateSpace* space, function<Waypoint()> sample):
		StateSampler {space},
		sample {sample},
		numDims {space->as<ob::RealVectorStateSpace>()->getDimension()}
	{}

	void sampleUniform(ob::State* state) override {
		auto s = sample();
		if (s.size() != numDims) {
			throw runtime_error("random state sampling function: sample size mismatch");
		}
		auto* rstate = static_cast<ob::RealVectorStateSpace::StateType*>(state);
		for (unsigned int i = 0; i < s.size(); ++i) {
			rstate->values[i] = s[i];
		}
	}

	void sampleUniformNear(ob::State*, const ob::State*, double) override {
		throw runtime_error("sampleUniformNear is not supported for CustomSampler");
	}
	void sampleGaussian(ob::State*, const ob::State*, double) override {
		throw runtime_error("sampleGaussian is not supported for CustomSampler");
	}
};



class CustomMotionValidator: public ob::MotionValidator {
	size_t nDims;
	function<bool(const Waypoint&, const Waypoint&)> callback;
	bool verbose;
public:
	CustomMotionValidator(
		function<bool(
			const Waypoint&,
			const Waypoint&
			)> callback,
		size_t nDims,
		bool verbose = false
		):
		MotionValidator {nullptr},
		nDims {nDims},
		callback {callback},
		verbose {verbose}
	{}

	bool checkMotion(const ob::State* s1, const ob::State* s2) const override {
		auto a1 = arrayFromState(s1, nDims);
		auto a2 = arrayFromState(s2, nDims);
		if (verbose) { cout << "checking " << a1 << " -> " << a2 << ":\n"; }
		bool result = callback(a1, a2);
		if (verbose) { cout << (result ? "ok" : "invalid") << "\n"; }
		if (result) { valid_++; }
		else { invalid_++; }
		return result;
	}

	bool checkMotion(
		const ob::State* s1,
		const ob::State* s2,
		std::pair<ob::State*, double>& lastValid
	) const override
	{
		(void) s1;
		(void) s2;
		(void) lastValid;
		throw runtime_error( "custom motion validation is not compatible with a planner that wants to know what fraction of the motion is possible");
	}
};


Path findPath(
	const Waypoint& start,
	const std::valarray<Waypoint>& targetPool,
	function<Waypoint()> sample,
	function<bool(const Waypoint&)> checkState,
	optional<function<bool(const Waypoint&, const Waypoint&)>> checkTransition,
	optional<function<double(const Waypoint&, const Waypoint&)>> distance,
	const string& planner,
	const std::map<std::string, std::string>& plannerParams,
	const std::valarray<Waypoint>& waypointSuggestions,
	double jiggle,
	size_t maxChecks,
	double timeout,
	bool verbose,
	function<void(const string&)> log
) {

	if (waypointSuggestions.size() > 0) {
		throw runtime_error("waypointSuggestions not implemented");
	}

	const size_t n = start.size();
	const size_t numTargets = targetPool.size();

	for (size_t t = 0; t < numTargets; t++) {
		if (targetPool[t].size() != n) {
			cout << "start:\n" << start << "\n";
			cout << "targetPool[" << t << "]:\n" << targetPool[t] << "\n";
			throw runtime_error("findPath: dimension mismatch");
		}
	}

	auto distanceFn = distance.value_or(
		[](const Waypoint& s0, const Waypoint& s1) {
			return sqrt(std::pow(s1 - s0, 2.0).sum());
		}
	);

	auto space = make_shared<ModdedRealVectorStateSpace>(n, distanceFn);

	auto isStateValid = [&](const ob::State* state) {
		auto values = arrayFromState(state, n);
		if (verbose) { cout << "checking " << values << ":\n"; }
		auto result = checkState(values);
		if (verbose) { cout << (result ? "ok" : "invalid") << "\n"; }
		return result;
	};

	ob::ScopedState<> obStart(space);
	vector<ob::ScopedState<>> obTargets;
	obTargets.reserve(numTargets);
	ob::RealVectorBounds bounds(n);

	for (size_t j = 0; j < n; j++) {
		obStart[j] = start[j];
		bounds.setLow(j, -inf);
		bounds.setHigh(j, inf);
	}
	for (size_t t = 0; t < numTargets; t++) {
		obTargets.push_back(ob::ScopedState<>(space));
		for (size_t j = 0; j < n; j++) {
			obTargets[t][j] = targetPool[t][j];
		}
	}

	// in case a configuration is on the edge of being valid, try to jiggle it free
	auto applyJiggle = [isStateValid, n, jiggle](ob::ScopedState<>& state) {
		if (isStateValid(state.get())) { return true; }
		else {
			for (size_t j = 0; j < n; j++) {
				for (double delta: {jiggle, -jiggle}) {
					state[j] += delta;
					if (isStateValid(state.get())) {
						return true;
					}
					state[j] -= delta;
				}
			}
		}
		return false;
	};

	space->setBounds(bounds);

	og::SimpleSetup ss(space);

	ss.setStateValidityChecker(isStateValid);

	if (plannerFactories.count(planner) == 0) {
		cout << "no planner named \"" << planner << "\"; "
			 << "available planners: ";
		for (const auto& p: plannerFactories) {
			cout << '"' << p.first << "\" ";
		}
		cout << "\n";
	}

	ss.setPlanner(plannerFactories.at(planner)(ss.getSpaceInformation()));
	auto asRRTC = std::dynamic_pointer_cast<og::RRTConnectCustom>(ss.getPlanner());
	if (asRRTC) { asRRTC->setLogger(log); }

	bool paramProblem = false;
	for (const auto& [p, v]: plannerParams) {
		if (not ss.getPlanner()->params().hasParam(p)) {
			cout << "planner " << planner << " has no parameter named \"" << p << "\"\n";
			paramProblem = true;
		}
	}

	if (paramProblem) {
		cout << "planner configuration:" << "\n";
		ss.getPlanner()->params().print(cout);
		throw runtime_error("invalid planner parameters");
	}

	bool success = ss.getPlanner()->params().setParams(plannerParams);

	if (not success) {
		throw runtime_error("error while parameterizing planner");
	}

	space->setStateSamplerAllocator(
		[&sample](const ob::StateSpace* space) {
			return make_shared<CustomSampler>(space, sample);
		}
	);

	if (checkTransition) {
		ss.getSpaceInformation()->setMotionValidator( make_shared<CustomMotionValidator>(checkTransition.value(), n, verbose));
	}
	else {
		ss.getSpaceInformation()->setStateValidityCheckingResolution(1.0 / 720.0); // ompl default is 0.01
	}

	bool startOk = applyJiggle(obStart);
	if (!startOk) {
		cout << "start configuration invalid, aborting" << "\n";
		return {};
	}
	ss.setStartState(obStart);

	auto goals = make_shared<ob::GoalStates>(ss.getSpaceInformation());
	int validTarget = -1;
	for (size_t i = 0; i < obTargets.size(); i++) {
		bool targetOk = applyJiggle(obTargets[i]);
		if (targetOk) {
			goals->addState(obTargets[i]);
			validTarget = i;
		}
	}
	if (goals->getStateCount() == 0) {
		cout << "no valid target configuration, aborting" << "\n";
		return {};
	}
	else if (goals->getStateCount() == 1) {
		ss.setGoalState(obTargets[validTarget]);
	}
	else {
		ss.setGoal(goals);
	}


	ss.setup();

	if (verbose) {
		// ss.print(); // already does lots of checks
	}

	Path result;

	// check if direct motion to the closest target is possible
	ob::ScopedState<> closestTarget(space);
	double closestDistance = std::numeric_limits<double>::infinity();
	for (const auto& target: obTargets) {
		double d = distanceFn(
			arrayFromState(&*obStart, n), arrayFromState(&*target, n));
		if (d < closestDistance) {
			closestDistance = d;
			closestTarget = target;
		}
	}

	if (ss.getSpaceInformation()->getMotionValidator()->checkMotion(
			&*obStart, &*closestTarget)) {

		result = {
			arrayFromState(&*obStart, n),
			arrayFromState(&*closestTarget, n)
		};
	}
	else {

		auto ptcIter = ob::IterationTerminationCondition(maxChecks);
		auto ptcTime = ob::timedPlannerTerminationCondition(timeout);
		auto ptcBoth = ob::plannerOrTerminationCondition(ptcIter, ptcTime);

		ob::PlannerTerminationCondition ptc =
			(timeout <= 0 || isnan(timeout) || isinf(timeout)) ?
			ptcIter : ptcBoth;

		ob::PlannerStatus solved = ss.solve(ptc);

		if (solved && ss.haveExactSolutionPath()) {
			auto& states = ss.getSolutionPath().getStates();

			result.resize(states.size());
			for (size_t i = 0; i < states.size(); i++) {
				result[i] = arrayFromState(states[i], n);
			}
		}

	}

	return result;
}
