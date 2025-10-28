
#pragma once

#include "collision/collisionchecker.h"
#include "collisionobject.h"
#include "shapestojson.h"

using nlohmann::json;

void to_json(json& j, const CollisionChecker& cc) {
	const auto& btObjects = cc.getCollisionWorld().getCollisionObjectArray();
	const auto& sceneInfo = cc.getSceneInfo();

	j = {};
	for (int i = 0; i < btObjects.size(); i++) {
		const btCollisionShape* btShape = btObjects[i]->getCollisionShape();
		collision::Shape* shape = collision::Shape::castFromBullet(btShape);

		j.push_back(json {
			{"name", CollisionObject::btCollisionObjectToString(btObjects[i])},
			{"shape", *shape},
			{"trafo", btObjects[i]->getWorldTransform()},
			{"globalGroups", sceneInfo[i].second.globalGroups.toString()},
			{"temporaryGroups", sceneInfo[i].second.temporaryGroups.toString()}
		});
	}
}

void to_json(json& j, const CollisionChecker::Collision& c) {
	j = {
		{"objectA", CollisionObject::btCollisionObjectToString(c.objectA)},
		{"objectB", CollisionObject::btCollisionObjectToString(c.objectB)},
		{"pointOnA", c.pointOnA},
		{"pointOnB", c.pointOnB}
	};
}
