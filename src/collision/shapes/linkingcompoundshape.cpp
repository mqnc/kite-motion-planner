
#include "linkingcompoundshape.h"

namespace collision {

LinkingCompoundShape::LinkingCompoundShape()
	: bulletShape {false}
// "false" argument disables dynamicAABBtree for children
// it only messed things up
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));
}

unique_ptr<Shape> LinkingCompoundShape::clone() const {
	auto other = make_unique<LinkingCompoundShape>();

	for (size_t i = 0; i < getNumShapes(); i++) {
		other->linkShape(
			const_cast<LinkingCompoundShape*>(this)->getShapeTrafo(i),
			const_cast<LinkingCompoundShape*>(this)->getShape(i)
		);
	}

	return other;
}

btCompoundShape* LinkingCompoundShape::getBulletShape() {
	return &bulletShape;
}

void LinkingCompoundShape::setSafetyMargin(double margin) {
	(void) margin;
	throw runtime_error("cannot set safety margin of linking compound shape, doesn't own children");
}

double LinkingCompoundShape::getSafetyMargin() const {
	throw runtime_error("cannot get safety margin of compound shape, can differ from child to child");
}


void LinkingCompoundShape::clear() {
	bulletShape = btCompoundShape(false);
	bulletShape.setUserPointer(static_cast<Shape*>(this));
}

void LinkingCompoundShape::linkShape(
	const btTransform& localTransform,
	Shape* shape
) {
	bulletShape.addChildShape(localTransform, shape->getBulletShape());
}

size_t LinkingCompoundShape::getNumShapes() const {
	return bulletShape.getNumChildShapes();
}

Shape* LinkingCompoundShape::getShape(size_t index) {
	return Shape::castFromBullet(bulletShape.getChildShape(index));
}

btTransform& LinkingCompoundShape::getShapeTrafo(size_t index) {
	return bulletShape.getChildTransform(index);
}

}
