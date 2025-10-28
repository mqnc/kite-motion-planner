#include "collisionchecker.h"

CollisionChecker::customCollisionConfiguration CollisionChecker::collisionConfiguration{};
btCollisionDispatcher CollisionChecker::dispatcher(&collisionConfiguration);

CollisionChecker::CollisionChecker(const SceneInfo& objects){

	PROFILE_SCOPE(CollisionChecker initialization);

	this->objects = objects;

	pairCache = make_unique<btHashedOverlappingPairCache>();
	filterCallback = make_unique<FilterCallback>(objects);
	broadphaseInterface = make_unique<btDbvtBroadphase>(pairCache.get());
	collisionWorld = make_unique<btCollisionWorld>(
		&dispatcher,
		broadphaseInterface.get(),
		&collisionConfiguration
	);
	pairCache->setOverlapFilterCallback(filterCallback.get());

	for (const auto& obj: objects) {
		collisionWorld->addCollisionObject(obj.first);
	}
}

CollisionChecker::CollisionReport CollisionChecker::checkCollisions(bool returnCollisions) {
	PROFILE_SCOPE(checkCollisions);

	collisionWorld->performDiscreteCollisionDetection();

	int numManifolds = collisionWorld->getDispatcher()->getNumManifolds();
	// int totalCandidates = 0;

	CollisionChecker::CollisionReport report;
	if (returnCollisions) {
		report.collisions = vector<Collision>();
	}

	for (int i = 0; i < numManifolds; i++) {
		btPersistentManifold* contactManifold =
			collisionWorld->getDispatcher()->getManifoldByIndexInternal(i);
		int numCandidates = contactManifold->getNumContacts();
		// totalCandidates += numCandidates;
		for (int j = 0; j < numCandidates; j++) {
			btManifoldPoint& pt = contactManifold->getContactPoint(j);
			if (pt.getDistance() < 0) {
				if (returnCollisions) {
					report.collisions->emplace_back(Collision {
						contactManifold->getBody0(),
						contactManifold->getBody1(),
						pt.getPositionWorldOnA(),
						pt.getPositionWorldOnB()
					});
				}
				report.numCollisions++;
			}
		}
	}

	return report;
}
