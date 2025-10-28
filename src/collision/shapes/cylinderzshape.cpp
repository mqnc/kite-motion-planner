
#include "cylinderzshape.h"

namespace collision {

CylinderZShape::CylinderZShape(double r, double h):
	bulletShape(btVector3(r, r, h / 2.0))
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));

	bulletShape.setMargin(0);
	setSafetyMargin(0);
}

unique_ptr<Shape> CylinderZShape::clone() const {
	auto other = make_unique<CylinderZShape>(getRadius(), getHeight());
	other->setSafetyMargin(getSafetyMargin());
	return other;
}

btCylinderShapeZ* CylinderZShape::getBulletShape() {
	return &bulletShape;
}

double CylinderZShape::getRadius() const {
	int radiusAxis = (bulletShape.getUpAxis() + 2) % 3;
	return bulletShape.getHalfExtentsWithoutMargin()[radiusAxis];
}

double CylinderZShape::getHeight() const {
	return bulletShape.getHalfExtentsWithoutMargin() // ...
			   [bulletShape.getUpAxis()] * 2.0;
}

void CylinderZShape::setSafetyMargin(double margin) {
	bulletShape.btConvexInternalShape::setMargin(margin);
}

double CylinderZShape::getSafetyMargin() const {
	return bulletShape.btConvexInternalShape::getMargin();
}

}
