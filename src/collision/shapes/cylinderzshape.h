
#pragma once

#include "shape.h"

namespace collision {

class CylinderZShape: public Shape {
	btCylinderShapeZ bulletShape;
public:
	CylinderZShape(double r, double h);
	unique_ptr<Shape> clone() const override;
	btCylinderShapeZ* getBulletShape() override;
	double getRadius() const;
	double getHeight() const;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
};

}
