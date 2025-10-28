
#include "collision/collisionchecker.h"
#include "stl.h"

// test what the collision margin on a bullet compound shape does

const int X = 0;
const int Y = 1;
const int Z = 2;

int main() {
  btConvexHullShape s1;
  s1.addPoint(btVector3(0, 0, 0));
  s1.setMargin(1);

  btConvexHullShape s2;
  s2.addPoint(btVector3(0, 0, 0));
  s2.setMargin(1);

  btCompoundShape cs{false};
  btTransform x10 = btTransform::getIdentity();
  x10.getOrigin()[X] = 10;
  cs.addChildShape(btTransform::getIdentity(), &s1);
  cs.addChildShape(x10, &s2);
  cs.setMargin(-0.5);

  btSphereShape floor{1};

  btCollisionObject o1;
  o1.setCollisionShape(&cs);

  btCollisionObject o2;
  o2.setCollisionShape(&floor);
  o2.getWorldTransform().getOrigin()[X] = 10;
  o2.getWorldTransform().getOrigin()[Z] = -1;

  for (double z = 2; z > 0; z -= 0.01) {
    o1.getWorldTransform().getOrigin()[Z] = z;

		BitMask bm0;
		bm0.set(0, false);

    CollisionChecker::SceneInfo scene = {{&o1, {bm0, bm0}}, {&o2, {bm0, bm0}}};

    std::cout << "z=" << z << ":"
              << CollisionChecker(scene).checkCollisions().numCollisions
              << "\n";
  }

  return 0;
}
