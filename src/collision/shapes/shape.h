
// Bullet physics uses a little margin around objects to make dynamic collision
// handling faster and more stable. We abuse this for creating a safety margin
// around objects so the trajectory planner plans with some wiggle room.
// However, bullet does not implement the margin in the same way for all shapes
// (see here https://www.youtube.com/watch?v=BGAwRKPlpCw) so we wrap them in a
// more consistent interface. Note that the box-box collision detection doesn't
// consider rounded edges (see video), replace with convex-convex if necessary.

#pragma once

#include "stl.h"
#include "bullet3/src/btBulletCollisionCommon.h"

namespace collision {

class Shape {
public:
	virtual unique_ptr<Shape> clone() const = 0;
	virtual btCollisionShape* getBulletShape() = 0;
	virtual void setSafetyMargin(double margin) = 0;
	virtual double getSafetyMargin() const = 0;
	virtual ~Shape() = 0;

	static Shape* castFromBullet(const btCollisionShape* const btShape);
};

}
