#pragma once

// the idea is that an api::scene may contain arbitrary kinds of transformations,
// unnormalized quaternions, invalid references etc, then comes a sanity check
// and a transformation to normalized transforms (dynamic rotation/translation or
// static transformation) and spits out a canonical::scene which is guaranteed to be
// valid.

#include "stl.h"
#include "../apiscene.h" // TODO bad design

namespace canonical {

using api::Raw;

using api::Vector3;
using api::Vector4;
using api::Axis;

using api::RefType;
using api::Reference;

struct Quaternion {
	Vector4 xyzw;
};

enum class TransformType { translation, rotation };

struct DynamicTransform {
	TransformType kind;
	Axis axis;
	Reference value;
};

struct StaticTransform {
	Vector3 translation = {0, 0, 0};
	Quaternion rotation = {0, 0, 0, 1};
};

using Transform = variant<StaticTransform, DynamicTransform>;
using TransformChain = vector<Transform>;

using api::ShapeType;
using api::Sphere;
struct Box {
	StaticTransform pose;
	Vector3 size;
};
using api::Cylinder;
using api::OrangeNet;
using api::Mesh;

struct Shape {
	variant<Sphere, Box, Cylinder, OrangeNet, Mesh> geometry;
	bool collides = true;
	double margin = 0.03;
	double stickyForce = NaN;
};

using api::Inertia;

struct Link {
	string parent = ROOT_ID;
	TransformChain pose;
	vector<string> parts;
};

struct RigidBody {
	double mass = 0.0;
	Vector3 centerOfMass = {0.0, 0.0, 0.0};
	Inertia inertia;
	Dict<Shape> shapes;
	Raw visuals;
};

using api::Joint;

struct Model {
	Dict<Link> links;
	Dict<RigidBody> partModels;
	Dict<Joint> joints;
	Dict<vector<string>> collisionIgnoreGroups;
	Dict<Dict<double>> constraints;
	Raw visuals;
};

struct Object {
	string model;
	string parent = WORLD_ID;
	TransformChain pose;
	Dict<variant<double, Reference>> jointValues;
};

using api::ControlPoint;
using api::Trajectory;

struct Scene {
	Dict<Model> models;
	Dict<Object> objects;
	Dict<vector<string>> collisionIgnoreGroups;
	Dict<Trajectory> trajectories;
};

} // namespace canonical
