
// compound shape owns its children

#pragma once

#include "shape.h"

namespace collision {

class CompoundShape: public Shape {
	btCompoundShape bulletShape;
	vector<unique_ptr<Shape>> children;
public:
	CompoundShape();
	CompoundShape(const CompoundShape& other);
	unique_ptr<Shape> clone() const override;

	btCompoundShape* getBulletShape() override;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;

	void clear();
	void addShape(
		const btTransform& localTransform,
		unique_ptr<Shape> shape
	);
	size_t getNumShapes() const;
	Shape* getShape(size_t index);
	btTransform& getShapeTrafo(size_t index);
};

}
