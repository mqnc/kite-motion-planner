
#pragma once

#include "shape.h"

namespace collision {

// all spheres have a (the same) radius, in addition to the safety margin
class InflatedConvexHullShape: public Shape {
	btConvexHullShape bulletShape;
	double radiusWithoutMargin = 0;
public:
	InflatedConvexHullShape(double radius);
	unique_ptr<Shape> clone() const override;
	btConvexHullShape* getBulletShape() override;
	double getRadius() const;
	void addPoint(double x, double y, double z);
	void addPoint(const btVector3& p);
	void optimize();
	size_t getNumPoints() const;
	btVector3 getPoint(size_t index) const;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
};

}
