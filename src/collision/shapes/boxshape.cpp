
#include "boxshape.h"

namespace collision {

BoxShape::BoxShape(double w, double h, double d):
	bulletShape {btVector3(w / 2.0, h / 2.0, d / 2.0)} // bullet uses half extents
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));

	bulletShape.setMargin(0);
	setSafetyMargin(0);
}

unique_ptr<Shape> BoxShape::clone() const {
	auto size = getSize();
	auto other = make_unique<BoxShape>(size.getX(), size.getY(), size.getZ());
	other->setSafetyMargin(getSafetyMargin());
	return other;
}

btBoxShape* BoxShape::getBulletShape() {
	return &bulletShape;
}

btVector3 BoxShape::getSize() const {
	return bulletShape.getHalfExtentsWithoutMargin() * 2.0;
}

void BoxShape::setSafetyMargin(double margin) {
	bulletShape.btConvexInternalShape::setMargin(margin);
}

double BoxShape::getSafetyMargin() const {
	return bulletShape.btConvexInternalShape::getMargin();
}

}
