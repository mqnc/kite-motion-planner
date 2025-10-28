
#include "compoundshape.h"

namespace collision {

CompoundShape::CompoundShape()
	: bulletShape(false) // disable dynamicAABBtree for children
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));

	setSafetyMargin(0);
}

CompoundShape::CompoundShape(const CompoundShape& other)
	: CompoundShape()
{
	for (size_t i = 0; i < other.getNumShapes(); i++) {
		addShape(
			const_cast<CompoundShape&>(other).getShapeTrafo(i),
			const_cast<CompoundShape&>(other).getShape(i)->clone()
		);
	}
}

unique_ptr<Shape> CompoundShape::clone() const {
	auto other = make_unique<CompoundShape>();

	for (size_t i = 0; i < getNumShapes(); i++) {
		other->addShape(
			const_cast<CompoundShape*>(this)->getShapeTrafo(i),
			const_cast<CompoundShape*>(this)->getShape(i)->clone()
		);
	}

	return other;
}

btCompoundShape* CompoundShape::getBulletShape() {
	return &bulletShape;
}

void CompoundShape::setSafetyMargin(double margin) {
	for (size_t i = 0; i < getNumShapes(); i++) {
		getShape(i)->setSafetyMargin(margin);
	}
	bulletShape.recalculateLocalAabb();
}

double CompoundShape::getSafetyMargin() const {
	throw runtime_error("cannot get safety margin of compound shape, "
						"can differ from child to child");
}

void CompoundShape::clear() {
	bulletShape = btCompoundShape(false);
	bulletShape.setUserPointer(static_cast<Shape*>(this));
	children.clear();
}

void CompoundShape::addShape(
	const btTransform& localTransform,
	unique_ptr<Shape> shape
) {
	children.push_back(std::move(shape));
	bulletShape.addChildShape(localTransform, children.back()->getBulletShape());
}

size_t CompoundShape::getNumShapes() const {
	return children.size();
}

Shape* CompoundShape::getShape(size_t index) {
	return children[index].get();
}

btTransform& CompoundShape::getShapeTrafo(size_t index) {
	return getBulletShape()->getChildTransform(index);
}

}
