#pragma once

#include "utils/profiling.h"
#include "stl.h"
#include "bitmask.h"

#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h"

struct CollisionIgnoreGroupBitMasks {
	BitMask globalGroups;
	BitMask temporaryGroups;
};

class CollisionChecker {

public:
	using SceneInfo = vector<pair<btCollisionObject*, CollisionIgnoreGroupBitMasks>>;

private:
	struct FilterCallback: public btOverlapFilterCallback {
		const SceneInfo& objects;

		FilterCallback(const SceneInfo& objects):
			objects {objects} {}

		virtual bool needBroadphaseCollision(
			btBroadphaseProxy* proxy0,
			btBroadphaseProxy* proxy1
		) const {

			int i0 = static_cast<btCollisionObject*>(proxy0->m_clientObject)->getWorldArrayIndex();
			int i1 = static_cast<btCollisionObject*>(proxy1->m_clientObject)->getWorldArrayIndex();

			return
				(objects[i0].second.globalGroups & objects[i1].second.globalGroups) == false
				&& (objects[i0].second.temporaryGroups & objects[i1].second.temporaryGroups) == false;
		}
	};

	class customCollisionConfiguration: public btDefaultCollisionConfiguration {
	public:
#ifdef COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
		customCollisionConfiguration() {
			// remove bullets box box algorithm (which doesn't consider rounded edges for inflated objects)
			m_boxBoxCF->~btCollisionAlgorithmCreateFunc();
			btAlignedFree(m_boxBoxCF);

			// and replace it with bullets convex convex algorithm, which will round inflated edges
			void* mem = NULL;
			mem = btAlignedAlloc(sizeof(btConvexConvexAlgorithm::CreateFunc), 16);
			m_boxBoxCF = new (mem) btConvexConvexAlgorithm::CreateFunc(m_pdSolver);
		}
#endif
	};

	// profiling showed that these two eat a lot of time when initialized again for each scene
	static customCollisionConfiguration collisionConfiguration;
	static btCollisionDispatcher dispatcher;
	
	unique_ptr<btOverlappingPairCache> pairCache;
	unique_ptr<FilterCallback> filterCallback;
	unique_ptr<btDbvtBroadphase> broadphaseInterface;
	unique_ptr<btCollisionWorld> collisionWorld;

	SceneInfo objects;

public:
	CollisionChecker(
		const SceneInfo& objects
	);

	struct Collision {
		const btCollisionObject* objectA = nullptr;
		const btCollisionObject* objectB = nullptr;
		btVector3 pointOnA;
		btVector3 pointOnB;
	};

	struct CollisionReport {
		size_t numCollisions = 0;
		optional<vector<Collision>> collisions;
	};

	CollisionReport checkCollisions(
		bool returnCollisions = false
	);

	const btCollisionWorld& getCollisionWorld() const {
		return *collisionWorld;
	}

	const SceneInfo& getSceneInfo() const {
		return objects;
	}
};
