
#pragma once

#include "shape.h"

namespace collision {

class BoxShape: public Shape {
	btBoxShape bulletShape;
public:
	BoxShape(double w, double h, double d);
	unique_ptr<Shape> clone() const override;
	btBoxShape* getBulletShape() override;
	btVector3 getSize() const;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
};

}
