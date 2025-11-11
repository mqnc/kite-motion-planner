/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "RRTConnectCustom.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/String.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <bitset>

ompl::geometric::RRTConnectCustom::RRTConnectCustom( const base::SpaceInformationPtr& spaceInformation, bool addIntermediateStates)
	: base::Planner(spaceInformation, addIntermediateStates ? "RRTConnectIntermediate" : "RRTConnectCustom")
{
	specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
	specs_.directed = true;

	Planner::declareParam<double>( "range", this, &RRTConnectCustom::setRange, &RRTConnectCustom::getRange, "0.:1.:10000.");
	Planner::declareParam<bool>("intermediate_states", this, &RRTConnectCustom::setIntermediateStates,
		&RRTConnectCustom::getIntermediateStates, "0,1");
	Planner::declareParam<size_t>("max_actuated_params", this, &RRTConnectCustom::setMaxActuatedParams,
		&RRTConnectCustom::getMaxActuatedParams, "0:1:1000");
    Planner::declareParam<size_t>("add_aabb_corners", this, &RRTConnectCustom::setAddAABBCorners,
		&RRTConnectCustom::getAddAABBCorners, "0,1");


	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
	distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
	addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RRTConnectCustom::~RRTConnectCustom()
{
	freeMemory();
}

void ompl::geometric::RRTConnectCustom::setup()
{
	Planner::setup();
	tools::SelfConfig sc(si_, getName());
	sc.configurePlannerRange(maxDistance_);

	if (!startTreeData_)
		startTreeData_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	if (!goalTreeData_)
		goalTreeData_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion*>(this));
	startTreeData_->setDistanceFunction( [this](const Motion* a, const Motion* b) { return distanceFunction(a, b); });
	goalTreeData_->setDistanceFunction([this](const Motion* a, const Motion* b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRTConnectCustom::freeMemory()
{
	std::vector<Motion*> motions;

	if (startTreeData_)
	{
		startTreeData_->list(motions);
		for (auto& motion: motions)
		{
			if (motion->state != nullptr)
				si_->freeState(motion->state);
			delete motion;
		}
	}

	if (goalTreeData_)
	{
		goalTreeData_->list(motions);
		for (auto& motion: motions)
		{
			if (motion->state != nullptr)
				si_->freeState(motion->state);
			delete motion;
		}
	}
}

void ompl::geometric::RRTConnectCustom::clear()
{
	Planner::clear();
	sampler_.reset();
	freeMemory();
	if (startTreeData_)
		startTreeData_->clear();
	if (goalTreeData_)
		goalTreeData_->clear();
	connectionPoint_ = std::make_pair<base::State*, base::State*>(nullptr, nullptr);
	distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::geometric::RRTConnectCustom::GrowProgress ompl::geometric::RRTConnectCustom::growTree(TreeData& tree, TreeGrowingInfo& treeGrowingInfo,
	Motion* randomMotion)
{
	/* find closest state in the tree */
	Motion* nearestMotion = tree->nearest(randomMotion);

	// auto n = si_->getStateDimension();
	// auto print = [n](base::State* s) {
	// 	auto rs = s->as<ompl::base::RealVectorStateSpace::StateType>();
	// 	for (size_t i = 0; i < n; i++) { std::cout << (*rs)[i] << "  "; }
	// 	std::cout << "\n";
	// };

	/* assume we can reach the state we go towards */
	bool reach = true;

	/* find state to add */
	base::State* dstate = randomMotion->state;
	double d = si_->distance(nearestMotion->state, randomMotion->state);

	// std::cout << "random1: "; print(randomMotion->state);
	// std::cout << "nearest1: "; print(nearestMotion->state);
	// std::cout << "dstate1: "; print(dstate);

    std::vector<uint8_t> actuated;
	auto nDims = si_->getStateDimension();
    actuated.reserve(nDims);

    using VectorState = ompl::base::RealVectorStateSpace::StateType;

	if (maxActuatedParams_ > 0) {
		if (maxActuatedParams_ < nDims) {
			auto k = maxActuatedParams_;
			const auto* start = nearestMotion->state->as<VectorState>();
			auto* target = dstate->as<VectorState>();
			auto* tgix_rv = treeGrowingInfo.xstate->as<VectorState>();

			std::vector<double> deltae;
			deltae.reserve(nDims);
			for (size_t i = 0; i < nDims; i++) {
				deltae.push_back(abs((*target)[i] - (*start)[i]));
			}

			// Find the k-th largest element
			std::nth_element(
				deltae.begin(),
				deltae.begin() + k - 1,
				deltae.end(),
				std::greater {});

			const auto& limit = deltae[k - 1];

			for (size_t i = 0; i < nDims; ++i) {
				double delta = abs((*target)[i] - (*start)[i]);
				if (delta < limit && delta > 1e-9) { // we need to reach the target if the step is close to zero
					(*tgix_rv)[i] = (*start)[i];
					reach = false;
				}
				else{
					(*tgix_rv)[i] = (*target)[i];
                    if (delta > 1e-9){
                        actuated.push_back(i);
                    }
				}
			}
			dstate = tgix_rv;
		}
	}
    else{
        for (size_t i = 0; i < nDims; i++) {
            actuated.push_back(i);
        }        
    }

	// std::cout << "random2: "; print(randomMotion->state);
	// std::cout << "nearest2: "; print(nearestMotion->state);
	// std::cout << "dstate2: "; print(dstate);

	d = si_->distance(nearestMotion->state, dstate);
	if (d > maxDistance_)
	{
		si_->getStateSpace()->interpolate(nearestMotion->state, dstate, maxDistance_ / d, treeGrowingInfo.xstate);

		/* Check if we have moved at all. Due to some stranger state spaces (e.g., the constrained state spaces),
		 * interpolate can fail and no progress is made. Without this check, the algorithm gets stuck in a loop as it *
 thinks it is making progress, when none is actually occurring. */
		if (si_->equalStates(nearestMotion->state, treeGrowingInfo.xstate))
			return TRAPPED;

		dstate = treeGrowingInfo.xstate;
		reach = false;
	}

	bool validMotion = treeGrowingInfo.isStartTree ? si_->checkMotion(nearestMotion->state, dstate) :
		si_->isValid(dstate) && si_->checkMotion(dstate, nearestMotion->state);

	if (!validMotion)
		return TRAPPED;

    if (addAABBCorners_ > 0)
    {
        // the user must guarantee that the whole AABB of the tested motion is valid
        // we can then add more corners from that AABB
        const auto* start = nearestMotion->state->as<VectorState>();
        auto* target = dstate->as<VectorState>();

        std::vector<size_t> combinations; // bit==0 in a combination: take value from start, 1: from target
        size_t count = (1UL << actuated.size()) - 1; // we don't add the start state 00000000
        combinations.reserve(count);

        for(size_t iCombi=1; iCombi < count+1; iCombi++) {
            size_t combination = 0;
            for(uint8_t iActuatedBit=0; iActuatedBit<actuated.size(); iActuatedBit++) {
                if((iCombi & (1 << iActuatedBit)) != 0) {
                    combination += 1 << actuated[iActuatedBit];
                }
            }
            combinations.push_back(combination);
        }

        if (addAABBCorners_ < count){
            static std::default_random_engine rng{addAABBCorners_}; // abuse the number of corners to add as random seed
            std::shuffle(combinations.begin(), combinations.end(), rng);
        }

        for (size_t iCombi=0; iCombi < std::min(addAABBCorners_, count); iCombi++){
            size_t combination = combinations[iCombi];
            VectorState* aabbCorner = si_->allocState()->as<VectorState>();

            for (size_t i=0; i < nDims; i++){
                if ((combination & (1 << i)) != 0) {
                    (*aabbCorner)[i] = (*target)[i];
                }
                else{
                    (*aabbCorner)[i] = (*start)[i];
                }
            }

            auto* motion = new Motion;
            motion->state = aabbCorner;
            motion->parent = nearestMotion;
            motion->root = nearestMotion->root;
            tree->add(motion);
        }
    }

	if (addIntermediateStates_)
	{
		const base::State* astate = treeGrowingInfo.isStartTree ? nearestMotion->state : dstate;
		const base::State* bstate = treeGrowingInfo.isStartTree ? dstate : nearestMotion->state;

		std::vector<base::State*> states;
		const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);

		if (si_->getMotionStates(astate, bstate, states, count, true, true))
			si_->freeState(states[0]);

		for (std::size_t i = 1; i < states.size(); ++i)
		{
			auto* motion = new Motion;
			motion->state = states[i];
			motion->parent = nearestMotion;
			motion->root = nearestMotion->root;
			tree->add(motion);

			nearestMotion = motion;
		}

		treeGrowingInfo.xmotion = nearestMotion;
	}
	else
	{
		auto* motion = new Motion(si_);
		si_->copyState(motion->state, dstate);
		motion->parent = nearestMotion;
		motion->root = nearestMotion->root;
		tree->add(motion);

		treeGrowingInfo.xmotion = motion;
	}

	return reach ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::geometric::RRTConnectCustom::solve( const base::PlannerTerminationCondition& terminationCondition)
{
	checkValidity();
	auto* goal = dynamic_cast<base::GoalSampleableRegion*>(pdef_->getGoal().get());

	if (goal == nullptr)
	{
		OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
		return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
	}

	while (const base::State* st = pis_.nextStart())
	{
		auto* motion = new Motion(si_);
		si_->copyState(motion->state, st);
		motion->root = motion->state;
		startTreeData_->add(motion);
	}

	if (startTreeData_->size() == 0)
	{
		OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
		return base::PlannerStatus::INVALID_START;
	}

	if (!goal->couldSample())
	{
		OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
		return base::PlannerStatus::INVALID_GOAL;
	}

	if (!sampler_)
		sampler_ = si_->allocStateSampler();

	OMPL_INFORM("%s: Starting planning with %d starts and %d targets already in datastructure", getName().c_str(),
		(int) (startTreeData_->size()), (int) (goalTreeData_->size()));

	TreeGrowingInfo treeGrowingInfo;
	treeGrowingInfo.xstate = si_->allocState();

	Motion* approxsol = nullptr;
	double approxdif = std::numeric_limits<double>::infinity();
	auto* randomMotion = new Motion(si_);
	bool solved = false;

	while (!terminationCondition)
	{
		log(isStartTree_ ? "start" : "goal");
		TreeData& thisTree = isStartTree_ ? startTreeData_ : goalTreeData_;
		treeGrowingInfo.isStartTree = isStartTree_;
		isStartTree_ = !isStartTree_;
		TreeData& otherTree = isStartTree_ ? startTreeData_ : goalTreeData_;

		if (goalTreeData_->size() == 0 || pis_.getSampledGoalsCount() < goalTreeData_->size() / 2)
		{
			const base::State* st = goalTreeData_->size() == 0 ? pis_.nextGoal(terminationCondition) : pis_.nextGoal();
			if (st != nullptr)
			{
				auto* motion = new Motion(si_);
				si_->copyState(motion->state, st);
				motion->root = motion->state;
				goalTreeData_->add(motion);
			}

			if (goalTreeData_->size() == 0)
			{
				OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
				break;
			}
		}

		/* sample random state */
		sampler_->sampleUniform(randomMotion->state);

		GrowProgress thisTreeProgress = growTree(thisTree, treeGrowingInfo, randomMotion);

		if (thisTreeProgress != TRAPPED)
		{
			/* remember which motion was just added */
			Motion* addedMotion = treeGrowingInfo.xmotion;

			/* attempt to connect trees */

			/* if reached, it means we used randomMotion->state directly, no need to copy again */
			if (thisTreeProgress != REACHED)
				si_->copyState(randomMotion->state, treeGrowingInfo.xstate);

			treeGrowingInfo.isStartTree = isStartTree_;

			/* if initial progress cannot be done from the otherTree, restore treeGrowingInfo.isStartTree */
			GrowProgress otherTreeProgress = growTree(otherTree, treeGrowingInfo, randomMotion);
			if (otherTreeProgress == TRAPPED)
				treeGrowingInfo.isStartTree = !treeGrowingInfo.isStartTree;

			while (otherTreeProgress == ADVANCED)
				otherTreeProgress = growTree(otherTree, treeGrowingInfo, randomMotion);

			/* update distance between trees */
			const double newDist = thisTree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
			if (newDist < distanceBetweenTrees_)
			{
				distanceBetweenTrees_ = newDist;
				// OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
			}

			Motion* startMotion = treeGrowingInfo.isStartTree ? treeGrowingInfo.xmotion : addedMotion;
			Motion* goalMotion = treeGrowingInfo.isStartTree ? addedMotion : treeGrowingInfo.xmotion;

			/* if we connected the trees in a valid way (start and goal pair is valid)*/
			if (otherTreeProgress == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
			{
				// it must be the case that either the start tree or the goal tree has made some progress
				// so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
				// on the solution path
				if (startMotion->parent != nullptr)
					startMotion = startMotion->parent;
				else
					goalMotion = goalMotion->parent;

				connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

				/* construct the solution path */
				Motion* solution = startMotion;
				std::vector<Motion*> mpath1;
				while (solution != nullptr)
				{
					mpath1.push_back(solution);
					solution = solution->parent;
				}

				solution = goalMotion;
				std::vector<Motion*> mpath2;
				while (solution != nullptr)
				{
					mpath2.push_back(solution);
					solution = solution->parent;
				}

				auto path(std::make_shared<PathGeometric>(si_));
				path->getStates().reserve(mpath1.size() + mpath2.size());
				for (int i = mpath1.size() - 1; i >= 0; --i)
					path->append(mpath1[i]->state);
				for (auto& i: mpath2)
					path->append(i->state);

				pdef_->addSolutionPath(path, false, 0.0, getName());
				solved = true;
				break;
			}
			else
			{
				// We didn't reach the goal, but if we were extending the start
				// tree, then we can mark/improve the approximate path so far.
				if (treeGrowingInfo.isStartTree)
				{
					// We were working from the startTree.
					double dist = 0.0;
					goal->isSatisfied(treeGrowingInfo.xmotion->state, &dist);
					if (dist < approxdif)
					{
						approxdif = dist;
						approxsol = treeGrowingInfo.xmotion;
					}
				}
			}
		}
	}

	si_->freeState(treeGrowingInfo.xstate);
	si_->freeState(randomMotion->state);
	delete randomMotion;

	OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), startTreeData_->size() + goalTreeData_->size(),
		startTreeData_->size(), goalTreeData_->size());

	if (approxsol && !solved)
	{
		/* construct the solution path */
		std::vector<Motion*> mpath;
		while (approxsol != nullptr)
		{
			mpath.push_back(approxsol);
			approxsol = approxsol->parent;
		}

		auto path(std::make_shared<PathGeometric>(si_));
		for (int i = mpath.size() - 1; i >= 0; --i)
			path->append(mpath[i]->state);
		pdef_->addSolutionPath(path, true, approxdif, getName());
		return base::PlannerStatus::APPROXIMATE_SOLUTION;
	}

	return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::RRTConnectCustom::getPlannerData(base::PlannerData& data) const
{
	Planner::getPlannerData(data);

	std::vector<Motion*> motions;
	if (startTreeData_)
		startTreeData_->list(motions);

	for (auto& motion: motions)
	{
		if (motion->parent == nullptr)
			data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
		else
		{
			data.addEdge( base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
		}
	}

	motions.clear();
	if (goalTreeData_)
		goalTreeData_->list(motions);

	for (auto& motion: motions)
	{
		if (motion->parent == nullptr)
			data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
		else
		{
			// The edges in the goal tree are reversed to be consistent with start tree
			data.addEdge( base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
		}
	}

	// Add the edge connecting the two trees
	data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

	// Add some info.
	data.properties["approx goal distance REAL"] = ompl::toString(distanceBetweenTrees_);
}
