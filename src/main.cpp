
#include "stl.h"
#include "str.h"
#include "interface/sceneio.h"
#include "interface/preprocessor/converter.h"
#include "interface/planningrequest.h"
#include "sweepingtransform.h"
#include "sweepingshape.h"
#include "scenegraph.h"
#include "timer.h"
#include "collision/shapes/allshapes.h"
#include "pathfinder/planners.hpp"
#include "pathfinder/findpath.h"
#include "pathtools/subdividetransitioncheck.hpp"
#include "pathtools/verify.hpp"
#include "pathtools/simplify.hpp"
#include "pathtools/tighten.hpp"
#include "pathtools/smoothen.hpp"
#include "utils/profiling.h"
#include "utils/tracer.h"
#include "node.h"
#include "subspace.hpp"

#include "utils/divideandconquershuffle.h"

// to visualize the result:

// - open tools/scene_visualizer.html via webserver (python3 -m http.server) in a browser
// - drop the result file into it

// todos if i have time:

// write a validity checker,
// using json5, populating all defaults, checking all sanities,
// also translate this to javascript using emscripten

// we need to be able to link other json files from json files

// smoothing

// proper joint weight handling

// verify requests

// use "__inf__", "__-inf__" and "__nan__" in classic json

// have a clear policy on vector vs valarray

// checkTransition assumes the start of a transition is always valid, which is OMPL policy
// not checking the start state gave a significant time boost, should also exclude the target when growing the target tree
// for start -> target, RRT does checkTransition,
// for target -> start, RRT does checkState on start, then checkTransition, which is unnecessary for sweeping

#define PRINT_PATH(PATH) \
std::cout << "\n\n" #PATH ":" << "\n"; for (const auto& p: PATH) { std::cout << "{ \"f\": " << p << ", \"df\": [0, 0, 0, 0, 0, 0] }," << "\n"; }

int main(int argc, char* argv[]) {
	vector<string> args(argv, argv + argc);

	if (args.size() < 4) {
		std::cout << "usage: " << args[0] << " scene.json request.json output.json" << std::endl;
		return EXIT_FAILURE;
	}

	auto apiScene = api::loadScene(args[1]);
	auto request = api::loadRequest(args[2]);
	auto outputFileName = args[3];

	auto scene = preprocess(apiScene);
	auto sceneGraph = SceneGraph(scene);

	size_t numJoints = request.actuatedJoints.size();

	vector<Observable<SweepingValue>*> joints;
	joints.reserve(numJoints);
	valarray<double> minJointValues(numJoints);
	valarray<double> maxJointValues(numJoints);

	auto expand = [numJoints](variant<double, valarray<double>>& v) {
		return holds_alternative<double>(v) ?
			valarray<double>(get<double>(v), numJoints) : get<valarray<double>>(v);
	};

	SweepMode sweepMode;
	valarray<double> maxStepSize;
	switch (request.motionTest) {
		case api::MotionTest::sampling:
			maxStepSize = expand(request.maxStepSize.sampling);
			sweepMode = SweepMode::unset;
			break;
		case api::MotionTest::cartesianLinearSweep:
			maxStepSize = expand(request.maxStepSize.cartesianLinearSweep);
			sweepMode = SweepMode::cartesianLinearSweep;
			break;
		case api::MotionTest::jointCuboidSweep:
			maxStepSize = expand(request.maxStepSize.jointCuboidSweep);
			sweepMode = SweepMode::jointCuboidSweep;
			break;
	}

	valarray<array<double, 2>> jointLimits(numJoints);

	for (size_t j = 0; j < numJoints; j++) {
		joints.push_back(&sceneGraph.referenceables.at("joint:" + request.actuatedJoints[j]));
		auto parts = str::split(request.actuatedJoints[j], '.');
		if (parts.size() != 2) { throw std::runtime_error("joint name must be of the form robotName.jointName"); }
		string robotName = parts[0];
		string jointName = parts[1];
		string modelName = scene.objects.at(robotName).model;
		auto joint = scene.models.at(modelName).joints.at(jointName);
		jointLimits[j] = {joint.minValue, joint.maxValue};
	}

	Subspace subspace {request.start, request.constraints, jointLimits, 0};

	auto checkState = [&](const Waypoint& jointValues){
		for (size_t j = 0; j < numJoints; j++) {
			if (jointValues[j] < jointLimits[j][0] || jointValues[j] > jointLimits[j][1]) {
				return false;
			}
		}
		for (size_t j = 0; j < numJoints; j++) {
			joints[j]->update(sweepMode, jointValues[j]);
		}
		bool result = sceneGraph.isCollisionFree(request.debugLevel);
		return result;
	};

	auto checkSubspaceState = [&](const Waypoint& params) {
		return checkState(subspace.unproject(params));
	};

	auto checkCuboid = [&](const Waypoint& jointValues0, const Waypoint& jointValues1) {
		for (size_t j = 0; j < numJoints; j++) {
			joints[j]->update(SweepMode::jointCuboidSweep,
				jointValues0[j],
				jointValues1[j]
			);
		}
		return sceneGraph.isCollisionFree(request.debugLevel);
	};

	auto checkPath = [&](const Path& path) {
		
		std::valarray<double> minValues = path[0];
		std::valarray<double> maxValues = path[0];

		for (const auto& jointValues : path) {
			for (size_t j = 0; j < jointValues.size(); j++) {
				if (jointValues[j] < jointLimits[j][0] || jointValues[j] > jointLimits[j][1]) {
					return false;
				}
				minValues[j] = fmin(minValues[j], jointValues[j]);
				maxValues[j] = fmax(maxValues[j], jointValues[j]);
			}
		}

		switch (request.motionTest) {
			case api::MotionTest::cartesianLinearSweep: {
				size_t numSegments = path.size() - 1;
				auto order = divideAndConquerShuffle(numSegments);
				for (size_t k = 0; k < numSegments; k++) {
					for (size_t j = 0; j < numJoints; j++) {
						joints[j]->update(sweepMode,
							path[order[k]][j],
							path[order[k] + 1][j]
						);
					}
					if (!sceneGraph.isCollisionFree(request.debugLevel)) {
						return false;
					}
				}
			}
			case api::MotionTest::jointCuboidSweep: {
				// todo: bit ballsy to check the whole motion like that, probably needs to be split up
				return checkCuboid(minValues, maxValues);
			}
			case api::MotionTest::sampling: {
				// it is OMPL policy that the start of a transition is always valid, so k=0 can be skipped
				auto order = divideAndConquerShuffle(path.size());
				for (size_t k = 1; k < path.size(); k++) {
					for (size_t j = 0; j < numJoints; j++) {
						joints[j]->update(sweepMode, path[order[k]][j]);
					}
					if (!sceneGraph.isCollisionFree(request.debugLevel)) {
						return false;
					}
				}
			}
		}
		return true;
	};

	auto checkSubspaceTransition = [&](const Waypoint& params0, const Waypoint& params1) {
		auto jointValues0 = subspace.unproject(params0);
		auto jointValues1 = subspace.unproject(params1);
		for (size_t j = 0; j < numJoints; j++) {
			if (
				jointValues0[j] < jointLimits[j][0] || jointValues0[j] > jointLimits[j][1]
				|| jointValues1[j] < jointLimits[j][0] || jointValues1[j] > jointLimits[j][1]
			) {
				return false;
			}
		}

		Path waypoints = subdivide(jointValues0, jointValues1, maxStepSize);

		switch (request.motionTest) {
			case api::MotionTest::cartesianLinearSweep:
				[[fallthrough]];
			case api::MotionTest::jointCuboidSweep: {
				size_t numSegments = waypoints.size() - 1;
				auto order = divideAndConquerShuffle(numSegments);
				for (size_t k = 0; k < numSegments; k++) {
					for (size_t j = 0; j < numJoints; j++) {
						joints[j]->update(sweepMode,
							waypoints[order[k]][j],
							waypoints[order[k] + 1][j]
						);
					}
					if (!sceneGraph.isCollisionFree(request.debugLevel)) {
						return false;
					}
				}
			}
			case api::MotionTest::sampling: {
				// it is OMPL policy that the start of a transition is always valid, so k=0 can be skipped
				auto order = divideAndConquerShuffle(waypoints.size());
				for (size_t k = 1; k < waypoints.size(); k++) {
					for (size_t j = 0; j < numJoints; j++) {
						joints[j]->update(sweepMode, waypoints[order[k]][j]);
					}
					if (!sceneGraph.isCollisionFree(request.debugLevel)) {
						return false;
					}
				}
			}
		}
		return true;
	};

	const string actuate = "6";
	const string range = "0.05";

	// checkTransition(request.start, request.targetPool[0]);

	Path subPath = findPath({
		.start = subspace.project(request.start),
		.targetPool = subspace.project(request.targetPool),
		.sample = [&]() { return subspace.randomSubSample(); },
		.checkState = checkSubspaceState,
		.checkTransition = checkSubspaceTransition,
		.planner = "RRTConnectCustom",
		.plannerParams = {{"range", range}, {"max_actuated_params", actuate}},
		.maxChecks = request.maxChecks,
		.timeout = request.timeout,
		.verbose = true
	});
	cout << "raw: " << subPath.size() << " waypoints\n";

	if (request.simplify){
		cout << "simplifying path...\n";
		subPath = skipCollinear(subPath);
		cout << "after skipping: " << subPath.size() << " waypoints\n";
		subPath = shortcut(subPath, checkSubspaceTransition, false);
		cout << "after finding shortcuts: " << subPath.size() << " waypoints\n";
	}

	if (request.tighten){
		cout << "tightening path...\n";
		subPath = tightenPath(subPath, checkSubspaceTransition);
		cout << "tight!\n";
	}

	Path superSpacePath = subspace.unproject(subPath);
	Path finalPath;
	double dt;
	if (request.smoothen){
		dt = 0.01;
		finalPath = smoothenPath(superSpacePath, dt,
			valarray<double>(1.0, numJoints),
			valarray<double>(1.0, numJoints),
			valarray<double>(1.0, numJoints),
			checkPath,
			&subspace
		);
	}
	else{
		finalPath = superSpacePath;
		dt = 1.0;
	}

	for (size_t j = 0; j < numJoints; j++) {
		api::Trajectory solution;
		solution.controlPoints.reserve(finalPath.size());
		double t = 0.0;
		for (const auto& p: finalPath) {
			solution.controlPoints.push_back({t, p[j]});
			t += dt;
		}
		string trajName = "__solution_" + str::replace(request.actuatedJoints[j], ".", "_");
		apiScene.trajectories[trajName] = solution;

		auto parts = str::split(request.actuatedJoints[j], '.');
		string robotName = parts[0];
		string jointName = parts[1];
		apiScene.objects.at(robotName).jointValues.at(jointName) = api::Reference {api::RefType::trajectory, trajName};
	}

	api::saveScene(outputFileName, apiScene, request.debugLevel > 0);

	return EXIT_SUCCESS;
}
