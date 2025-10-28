
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

	auto checkState = [&](const Waypoint& params) {
		auto jointValues = subspace.unproject(params);
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

	auto checkTransition = [&](const Waypoint& params0, const Waypoint& params1) {
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

	checkTransition({0, -0.8, 1.4, -0.6, 0, 0}, {0.5236, -0.8, 1.4+0.5236, -0.6, 0, 0});
	return 0;

	const string actuate = "6";
	const string range = "0.05";

	// checkTransition(request.start, request.targetPool[0]);


	auto pathRaw = findPath({
		.start = subspace.project(request.start),
		.targetPool = subspace.project(request.targetPool),
		.sample = [&]() { return subspace.randomSubSample(); },
		.checkState = checkState,
		.checkTransition = checkTransition,
		.planner = "RRTConnectCustom",
		.plannerParams = {{"range", range}, {"max_actuated_params", actuate}},
		.maxChecks = request.maxChecks,
		.timeout = request.timeout,
		.verbose = true
	});
	cout << "raw: " << pathRaw.size() << " waypoints\n";

	auto pathSkipped = skipCollinear(pathRaw);

	cout << "skipped: " << pathSkipped.size() << " waypoints\n";

	auto pathShort = shortcut(pathSkipped, checkTransition, false);

	cout << "shortcut: " << pathShort.size() << " waypoints\n";

	// cout << pathSimp1.size() << " waypoints\n";
	// auto pathSimp2 = shortcut(pathRaw, checkTransition, false);
	// cout << pathSimp2.size() << " waypoints\n";
	// auto pathSimp3 = shortcut(pathRaw, subdivide(checkTransition, 8), true);
	// cout << pathSimp3.size() << " waypoints\n";
	// auto pathTight = tightenPath(pathSimp3, subdivide(checkTransition, 4));

	auto finalPath = subspace.unproject(pathShort);

	for (size_t j = 0; j < numJoints; j++) {
		api::Trajectory solution;
		solution.controlPoints.reserve(finalPath.size());
		double t = 0.0;
		for (const auto& p: finalPath) {
			solution.controlPoints.push_back({t++, p[j]});
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

// int main_simple() {
// 	auto scene = api::loadScene("../../scenes/robot_with_sword.json");
// 	auto canonicalScene = preprocess(scene);
// 	auto g = SceneGraph(canonicalScene);

// 	const int debugLevel = 2;

// 	//SweepMode mode = SweepMode::jointCuboidSweep;
// 	SweepMode mode = SweepMode::cartesianLinearSweep;

// 	vector<string> joints = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};

// 	//Waypoint start = {0, 0, 0, 0, 0, 0};
// 	Waypoint start = {0_deg, 0_deg, 0_deg, 0_deg, 0_deg, 0_deg};
// 	Waypoint target = {45_deg, 45_deg, 45_deg, 45_deg, 45_deg, 45_deg};

// 	for (size_t j = 0; j < joints.size(); j++) {
// 		g.referenceables.at("joint:robot." + joints[j]).update(mode, start[j]);
// 	}

// 	auto checkTransition = [&](const Waypoint& wp0, const Waypoint& wp1) {
// 		for (size_t j = 0; j < joints.size(); j++){
// 			g.referenceables.at("joint:robot." + joints[j]).update(mode, wp0[j], wp1[j]);
// 		}
// 		return g.isCollisionFree(debugLevel);
// 	};

// 	checkTransition(start, target);

// 	return 0;

// }


// int main_maze() {
// 	auto scene = api::loadScene("../../scenes/maze.json");
// 	auto canonicalScene = preprocess(scene);
// 	auto g = SceneGraph(canonicalScene);

// 	const int debugLevel = 0;

// 	bool verbose = false;
// 	bool tracing = true;
// 	//SweepMode mode = SweepMode::jointCuboidSweep;
// 	SweepMode mode = SweepMode::cartesianLinearSweep;

// 	Tracer tracer;

// 	string tracedNode = "robot.saber";
// 	Vector3 tracedPoint {0, 0.5, 0};

// 	vector<string> joints = {"base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};
// 	vector<int> actuatedJoints = {1, 2, 3};

// 	Waypoint allStart = {0, -0.7, 2.2, -1, 1.57, 1.57};
// 	Waypoint allTarget = {0, -1.9, 2.4, -1, 1.57, 1.57};

// 	Waypoint actuatedStart = {allStart[1], allStart[2], allStart[3]};
// 	Waypoint actuatedTarget = {allTarget[1], allTarget[2], allTarget[3]};

// 	for (size_t j = 0; j < joints.size(); j++) {
// 		g.referenceables.at("joint:robot." + joints[j]).update(mode, allStart[j]);
// 	}

// 	auto checkState = [&](const Waypoint& wp) {
// 		for (size_t j = 0; j < actuatedJoints.size(); j++) {
// 			g.referenceables.at("joint:robot." + joints[actuatedJoints[j]]).update(mode, wp[j]);
// 		}

// 		bool result = g.isCollisionFree(debugLevel);

// 		if (tracing) {
// 			g.computeForwardKinematics();
// 			tracer.append({{g.nodes.at(tracedNode).getForwardKinematicsResult() * tracedPoint}, result});
// 		}

// 		if (verbose) { cout << "    " << wp << (result ? " v\n" : " X\n"); }

// 		return result;
// 	};

// 	auto checkTransition = [&](const Waypoint& wp0, const Waypoint& wp1) {

// 		for (size_t j = 0; j < actuatedJoints.size(); j++) {
// 			g.referenceables.at("joint:robot." + joints[actuatedJoints[j]]).update(mode, wp0[j], wp1[j]);
// 		}

// 		bool result = g.isCollisionFree(debugLevel);

// 		if (tracing) {
// 			for (size_t j = 0; j < actuatedJoints.size(); j++) {
// 				g.referenceables.at("joint:robot." + joints[actuatedJoints[j]]).update(mode, wp0[j]);
// 			}
// 			g.computeForwardKinematics();
// 			Vector3 p0 = g.nodes.at(tracedNode).getForwardKinematicsResult() * tracedPoint;

// 			for (size_t j = 0; j < actuatedJoints.size(); j++) {
// 				g.referenceables.at("joint:robot." + joints[actuatedJoints[j]]).update(mode, wp1[j]);
// 			}
// 			g.computeForwardKinematics();
// 			Vector3 p1 = g.nodes.at(tracedNode).getForwardKinematicsResult() * tracedPoint;

// 			tracer.append({{p0, p1}, result});
// 		}

// 		if (verbose) { cout << "    " << wp0 << " -> " << wp1 << (result ? " v\n" : " X\n"); }

// 		return result;
// 	};

// 	auto findAndTweakPath = [&](const Waypoint& start, const Waypoint& target) {

// 		const string actuate = "2";
// 		const string range = "0.05";

// 		auto pathRaw = findPath({
// 			.start = start,
// 			.targets = {target},
// 			.min = valarray<double>(-5, actuatedJoints.size()),
// 			.max = valarray<double>(5, actuatedJoints.size()),
// 			.checkState = checkState,
// 			.checkTransition = checkTransition,
// 			.planner = "RRTConnectCustom",
// 			.plannerParams = {{"range", range}, {"max_actuated_params", actuate}},
// 			.maxChecks = 50000000,
// 			.timeout = 24 * 60 * 60,
// 			.verbose = true,
// 			.log = [&tracer](const string& s) { tracer.append(s); }//,
// 			//.randomSeed = 123
// 		});
// 		cout << pathRaw.size() << " waypoints\n";

// 		return pathRaw;

// 		auto pathSimp1 = shortcut(pathRaw, checkTransition, false);
// 		cout << pathSimp1.size() << " waypoints\n";
// 		auto pathSimp2 = shortcut(pathRaw, subdivide(checkTransition, 4), false);
// 		cout << pathSimp2.size() << " waypoints\n";
// 		auto pathSimp3 = shortcut(pathRaw, subdivide(checkTransition, 8), true);
// 		cout << pathSimp3.size() << " waypoints\n";
// 		auto pathTight = tightenPath(pathSimp3, subdivide(checkTransition, 4));
// 		return pathTight;
// 	};

// 	auto path = findAndTweakPath(actuatedStart, actuatedTarget);

// 	for (auto& p: path) {
// 		p = {allStart[0], p[0], p[1], p[2], allStart[4], allStart[5]};
// 	}

// 	PRINT_PATH(path);

// 	writeFile("mazetraces.json", json(tracer).dump());

// 	return 0;

// }


/*
int main_lightsabers() {

	auto infos = getPlannerInfo();
	for (const auto& [name, info]: infos) {
		std::cout << name << "\n";
		for (const auto& [param, data]: info.params) {
			std::cout << "\t" << param << ": "
					  << data.defaultValue << "; "
					  << data.rangeSuggestion << "\n";
		}
	}

#ifdef NDEBUG
	std::cout << "release mode detected\n";
#else
	std::cout << "debug mode detected\n";
#endif
#ifdef __OPTIMIZE__
	std::cout << "code optimization detected\n";
#else
	std::cout << "no code optimization detected\n";
#endif

	api::Scene scene1 = api::loadScene("../../teaser/ico_scene1_planning.json");
	api::Scene scene2 = api::loadScene("../../teaser/ico_scene2_planning.json");

	SceneGraph g(preprocess(scene1));

	const int debugLevel = 0;

	bool verbose = false;
	bool tracing = false;

	Tracer tracer;

	string tracedNode = "robot1.saber";
	Vector3 tracedPoint {0, 0.715, 0};

	auto checkState = [&](const Waypoint& j) {

		for (size_t i = 1; i <= 30; i++) {
			g.referenceables.at("joint:robot" + to_string(i) + ".base").update(j[0]);
			g.referenceables.at("joint:robot" + to_string(i) + ".shoulder").update(j[1]);
			g.referenceables.at("joint:robot" + to_string(i) + ".elbow").update(j[2]);
			g.referenceables.at("joint:robot" + to_string(i) + ".wrist1").update(j[3]);
			g.referenceables.at("joint:robot" + to_string(i) + ".wrist2").update(j[4]);
			g.referenceables.at("joint:robot" + to_string(i) + ".wrist3").update(j[5]);
		}

		bool result = g.isCollisionFree(debugLevel);

		if (tracing) {
			g.computeForwardKinematics();
			tracer.append({{g.nodes.at(tracedNode).getForwardKinematicsResult() * tracedPoint}, result});
		}

		if (verbose) { cout << "    " << j << (result ? " v\n" : " X\n"); }

		return result;
	};

	auto checkTransition = [&](const Waypoint& j0, const Waypoint& j1) {

		for (size_t i = 1; i <= 30; i++) {
			g.referenceables.at("joint:robot" + to_string(i) + ".base").update(j0[0], j1[0]);
			g.referenceables.at("joint:robot" + to_string(i) + ".shoulder").update(j0[1], j1[1]);
			g.referenceables.at("joint:robot" + to_string(i) + ".elbow").update(j0[2], j1[2]);
			g.referenceables.at("joint:robot" + to_string(i) + ".wrist1").update(j0[3], j1[3]);
			g.referenceables.at("joint:robot" + to_string(i) + ".wrist2").update(j0[4], j1[4]);
			g.referenceables.at("joint:robot" + to_string(i) + ".wrist3").update(j0[5], j1[5]);
		}

		bool result = g.isCollisionFree(debugLevel);

		if (tracing) {
			for (size_t i = 1; i <= 30; i++) {
				g.referenceables.at("joint:robot" + to_string(i) + ".base").update(j0[0]);
				g.referenceables.at("joint:robot" + to_string(i) + ".shoulder").update(j0[1]);
				g.referenceables.at("joint:robot" + to_string(i) + ".elbow").update(j0[2]);
				g.referenceables.at("joint:robot" + to_string(i) + ".wrist1").update(j0[3]);
				g.referenceables.at("joint:robot" + to_string(i) + ".wrist2").update(j0[4]);
				g.referenceables.at("joint:robot" + to_string(i) + ".wrist3").update(j0[5]);
			}
			g.computeForwardKinematics();
			Vector3 p0 = g.nodes.at(tracedNode).getForwardKinematicsResult() * tracedPoint;

			for (size_t i = 1; i <= 30; i++) {
				g.referenceables.at("joint:robot" + to_string(i) + ".base").update(j1[0]);
				g.referenceables.at("joint:robot" + to_string(i) + ".shoulder").update(j1[1]);
				g.referenceables.at("joint:robot" + to_string(i) + ".elbow").update(j1[2]);
				g.referenceables.at("joint:robot" + to_string(i) + ".wrist1").update(j1[3]);
				g.referenceables.at("joint:robot" + to_string(i) + ".wrist2").update(j1[4]);
				g.referenceables.at("joint:robot" + to_string(i) + ".wrist3").update(j1[5]);
			}
			g.computeForwardKinematics();
			Vector3 p1 = g.nodes.at(tracedNode).getForwardKinematicsResult() * tracedPoint;

			tracer.append({{p0, p1}, result});
		}

		if (verbose) { cout << "    " << j0 << " -> " << j1 << (result ? " v\n" : " X\n"); }

		return result;
	};

	auto findAndTweakPath = [&](const Waypoint& start, const Waypoint& target) {

		const string actuate = "2";
		const string range = "0.075";

		auto pathRaw = findPath({
			.start = start,
			.targets = {target},
			.min = {-5, -5, -5, -5, -5, -5},
			.max = {5, 5, 5, 5, 5, 5},
			.checkState = checkState,
			.checkTransition = checkTransition,
			.planner = "RRTConnectCustom",
			.plannerParams = {{"range", range}, {"max_actuated_params", actuate}},
			.maxChecks = 50000000,
			.timeout = 24 * 60 * 60,
			.verbose = true
		});
		cout << pathRaw.size() << " waypoints\n";

		auto pathSimp1 = shortcut(pathRaw, checkTransition, false);
		cout << pathSimp1.size() << " waypoints\n";
		auto pathSimp2 = shortcut(pathRaw, subdivide(checkTransition, 4), false);
		cout << pathSimp2.size() << " waypoints\n";
		auto pathSimp3 = shortcut(pathRaw, subdivide(checkTransition, 8), true);
		cout << pathSimp3.size() << " waypoints\n";
		auto pathTight = tightenPath(pathSimp3, subdivide(checkTransition, 4));
		return pathTight;
	};

	auto path1 = findAndTweakPath(
		{0.4930129482963367, -1.4677392247870031, 2.28359874796597, -2.3866558499738635, -1.5707963267948966, -1.0777833784985598},
		{-2.3857232293276303, -1.6977241853700686, -0.5086315535049133, 2.2063557388749833, 0.7558694242621619, 3.141592653589793}
	);

	g = SceneGraph(preprocess(scene2));

	auto path2 = findAndTweakPath(
		{-2.3857232293276303, -1.6977241853700686, -0.5086315535049133, 2.2063557388749833, 0.7558694242621619, 3.141592653589793},
		{-0.46042996076263343, -1.5267416976996122, 1.9963575002127105, 1.1011805242817985, 1.570796326794897, 2.6811626928271597}
	);

	PRINT_PATH(path1);
	PRINT_PATH(path2);

	writeFile("traces.json", json(tracer).dump());

	return 0;
}
*/

// int main() {
// 	return main_subspace();
// }
