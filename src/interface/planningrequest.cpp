
#include "planningrequest.h"
#include "json.hpp"

using json = nlohmann::json;


namespace nlohmann {
// the usual to_json from_json definitions don't work with namespaces (like std)
// https://github.com/nlohmann/json/issues/1261#issuecomment-438448529
template<>
struct adl_serializer<variant<double, valarray<double>>> {
	static void to_json(json& j, const variant<double, valarray<double>>& v) {
		std::visit([&j](const auto& value) { j = value; }, v);
	}
	static void from_json(const json& j, variant<double, valarray<double>>& v) {
		if(j.is_number()) {
			v = j.get<double>();
		} else {
			v = j.get<valarray<double>>();
		}
	};
};
}

namespace api {

NLOHMANN_JSON_SERIALIZE_ENUM(MotionTest, {
		{MotionTest::sampling, "sampling"},
		{MotionTest::cartesianLinearSweep, "cartesianLinearSweep"},
		{MotionTest::jointCuboidSweep, "jointCuboidSweep"},
	});

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
	MaxStepSize,
	sampling,
	cartesianLinearSweep,
	jointCuboidSweep,
	jointCuboidSubdivision,
    evaluationSampling
);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(
	PlanningRequest,
	motionTest,
	actuatedJoints,
	start,
	targetPool,
	constraints,
	maxStepSize,
	timeout,
	maxChecks,
    simplify,
    tighten,
    smoothen,
    RRTRange,
    RRTMaxActuatedJoints,
    optimizeConvexHulls,
    addAABBCorners,
    randomSeed,
	debugLevel,
    evaluateTransitionChecks
);

PlanningRequest loadRequest(const std::string& file) {
	std::ifstream f(file);
	json data = json::parse(f);
	return data.get<PlanningRequest>();
}

}
