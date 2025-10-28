
#pragma once

#include "stl.h"

struct PlannerParamInfo {
	string defaultValue;
	string rangeSuggestion;
	// string help; // going to scrape that info from the ompl source (todo)
};

struct PlannerInfo {
	bool multithreaded;
	bool directed;
	std::map<string, PlannerParamInfo> params;
};

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

// https://github.com/ompl/ompl/blob/main/py-bindings/headers_geometric.txt
#include <ompl/geometric/planners/prm/ConnectionStrategy.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/SPARS.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/est/BiEST.h>
#include <ompl/geometric/planners/est/ProjEST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/kpiece/LBKPIECE1.h>
#include <ompl/geometric/planners/pdst/PDST.h>
//#include <ompl/geometric/planners/quotientspace/algorithms/MultiQuotient.h>
//#include <ompl/geometric/planners/quotientspace/QRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/LazyRRT.h>
#include <ompl/geometric/planners/rrt/TRRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/LBTRRT.h>
#include <ompl/geometric/planners/rrt/LazyLBTRRT.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
//#include <ompl/geometric/planners/informedtrees/BITstar.h>
//#include <ompl/geometric/planners/informedtrees/ABITstar.h>
//#include <ompl/geometric/planners/informedtrees/AITstar.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/sst/SST.h>

#include "RRTConnectCustom.h"


#define LIST_PLANNER(P) \
{#P, [](const ompl::base::SpaceInformationPtr &si){ \
    return make_shared<ompl::geometric::P>(si); \
}}

const std::map<
	string,
	function<
	ompl::base::PlannerPtr(const ompl::base::SpaceInformationPtr& si)
	>
> plannerFactories{
	LIST_PLANNER(BFMT),
	LIST_PLANNER(BiEST),
	LIST_PLANNER(BKPIECE1),
	LIST_PLANNER(EST),
	LIST_PLANNER(FMT),
	LIST_PLANNER(KPIECE1),
	LIST_PLANNER(LazyLBTRRT),
	LIST_PLANNER(LazyPRM),
	LIST_PLANNER(LazyPRMstar),
	LIST_PLANNER(LazyRRT),
	LIST_PLANNER(LBKPIECE1),
	LIST_PLANNER(LBTRRT),
	LIST_PLANNER(PDST),
	LIST_PLANNER(PRM),
	LIST_PLANNER(PRMstar),
	LIST_PLANNER(ProjEST),
	LIST_PLANNER(RRT),
	LIST_PLANNER(RRTConnect),
	LIST_PLANNER(RRTConnectCustom),
	LIST_PLANNER(RRTsharp),
	LIST_PLANNER(RRTstar),
	LIST_PLANNER(RRTXstatic),
	LIST_PLANNER(SBL),
	LIST_PLANNER(SORRTstar),
	LIST_PLANNER(SPARS),
	LIST_PLANNER(SPARStwo),
	LIST_PLANNER(SST),
	LIST_PLANNER(STRIDE),
	LIST_PLANNER(TRRT)
};


inline std::map<string, PlannerInfo> getPlannerInfo() {
	auto space = make_shared<ompl::base::RealVectorStateSpace>(1);
	auto spaceInfo = make_shared<ompl::base::SpaceInformation>(space);

	std::map<string, PlannerInfo> result;

	for (const auto& [name, makePlanner] : plannerFactories) {
		auto planner = makePlanner(spaceInfo);

		PlannerInfo info;

		info.multithreaded = planner->getSpecs().multithreaded;
		info.directed = planner->getSpecs().directed;

		for (const auto& [key, param] : planner->params().getParams()) {
			info.params[key].defaultValue = param->getValue();
			info.params[key].rangeSuggestion = param->getRangeSuggestion();
		}

		result[name] = info;
	}

	return result;
}
