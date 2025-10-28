
#pragma once

#include "shape.h"

namespace collision {

class VisualMeshShape: public Shape {
	btEmptyShape bulletShape;
	vector<btVector3> vertices;
	vector<array<size_t, 3>> faces;

public:
	VisualMeshShape(
		vector<btVector3> vertices,
		vector<array<size_t, 3>> faces
	);
	unique_ptr<Shape> clone() const override;
	btEmptyShape* getBulletShape() override;
	void setSafetyMargin(double margin) override;
	double getSafetyMargin() const override;
	void addVertex(btVector3 v);
	void addFace(const array<size_t, 3>& f);
	const vector<btVector3>& getVertices() const;
	const vector<array<size_t, 3>>& getFaces() const;
};

}
