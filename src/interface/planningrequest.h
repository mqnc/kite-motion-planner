
#pragma once

#include "stl.h"

namespace api {

enum class MotionTest {
	sampling,
	cartesianLinearSweep,
	jointCuboidSweep
};

struct MaxStepSize {
	variant<double, valarray<double>> sampling = 1.01_deg;
	variant<double, valarray<double>> cartesianLinearSweep = 5.01_deg;
	variant<double, valarray<double>> jointCuboidSweep = 60.1_deg;
	variant<double, valarray<double>> jointCuboidSubdivision = 30.1_deg;
};

struct PlanningRequest {
	MotionTest motionTest = MotionTest::cartesianLinearSweep;
	vector<string> actuatedJoints;
	valarray<double> start;
	valarray<valarray<double>> targetPool;
	valarray<valarray<double>> constraints;
	MaxStepSize maxStepSize;
	int debugLevel = 0;
	double timeout = 5;
	size_t maxChecks = 10000;
};

PlanningRequest loadRequest(const std::string& file);

}
