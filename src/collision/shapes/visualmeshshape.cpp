
#include "visualmeshshape.h"

namespace collision {

VisualMeshShape::VisualMeshShape(
	vector<btVector3> vertices,
	vector<array<size_t, 3>> faces
	):
	vertices {vertices},
	faces {faces}
{
	bulletShape.setUserPointer(static_cast<Shape*>(this));
}

unique_ptr<Shape> VisualMeshShape::clone() const {
	return make_unique<VisualMeshShape>(getVertices(), getFaces());
}

btEmptyShape* VisualMeshShape::getBulletShape() {
	return &bulletShape;
}

void VisualMeshShape::setSafetyMargin(double margin) {
	(void) margin;
	throw runtime_error("VisualMeshShape can't have safety margin");
}

double VisualMeshShape::getSafetyMargin() const {
	throw runtime_error("VisualMeshShape can't have safety margin");
}

void VisualMeshShape::addVertex(btVector3 v) {
	vertices.emplace_back(v);
}

void VisualMeshShape::addFace(const array<size_t, 3>& f) {
	faces.emplace_back(f);
}

const vector<btVector3>& VisualMeshShape::getVertices() const {
	return vertices;
}

const vector<array<size_t, 3>>& VisualMeshShape::getFaces() const {
	return faces;
}

}
