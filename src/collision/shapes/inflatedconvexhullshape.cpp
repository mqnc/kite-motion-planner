
#include "inflatedconvexhullshape.h"

namespace collision {

InflatedConvexHullShape::InflatedConvexHullShape(double radius):
	radiusWithoutMargin {radius}
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));

	setSafetyMargin(0);
}

unique_ptr<Shape> InflatedConvexHullShape::clone() const {
	auto other = make_unique<InflatedConvexHullShape>(getRadius());

	for (size_t i = 0; i < getNumPoints(); i++) {
		other->addPoint(getPoint(i));
	}

	other->setSafetyMargin(getSafetyMargin());

	return other;
}

btConvexHullShape* InflatedConvexHullShape::getBulletShape() {
	return &bulletShape;
}

double InflatedConvexHullShape::getRadius() const {
	return radiusWithoutMargin;
}

void InflatedConvexHullShape::addPoint(double x, double y, double z) {
	bulletShape.addPoint(btVector3(x, y, z));
}

void InflatedConvexHullShape::addPoint(const btVector3& p) {
	bulletShape.addPoint(p);
}

void InflatedConvexHullShape::optimize() {
	bulletShape.optimizeConvexHull();
}

size_t InflatedConvexHullShape::getNumPoints() const {
	return bulletShape.getNumPoints();
}

btVector3 InflatedConvexHullShape::getPoint(size_t index) const {
	return bulletShape.getUnscaledPoints()[index];
}

void InflatedConvexHullShape::setSafetyMargin(double margin) {
	bulletShape.setMargin(radiusWithoutMargin + margin);
}

double InflatedConvexHullShape::getSafetyMargin() const {
	return bulletShape.getMargin() - radiusWithoutMargin;
}

}
