
#include "shape.h"

namespace collision {

Shape::~Shape() {};

Shape* Shape::castFromBullet(const btCollisionShape* const btShape) {
	assert(btShape->getUserPointer() != nullptr);
	auto shape = static_cast<Shape*>(btShape->getUserPointer());
	assert(shape->getBulletShape() == btShape);
	return shape;
}

}
