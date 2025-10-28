
// linking compound shape does not own its children

#pragma once

#include "shape.h"

namespace collision {

class LinkingCompoundShape: public Shape {
	btCompoundShape bulletShape;
public:
	LinkingCompoundShape();
	unique_ptr<Shape> clone() const override;

	btCompoundShape* getBulletShape() override;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;

	void clear();
	void linkShape(
		const btTransform& localTransform,
		Shape* shape
	);
	size_t getNumShapes() const;
	Shape* getShape(size_t index);
	btTransform& getShapeTrafo(size_t index);
};

}
