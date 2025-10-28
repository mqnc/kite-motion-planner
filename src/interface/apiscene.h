#pragma once

#include "stl.h"

#ifndef WORLD_ID
#	define WORLD_ID "WORLD"
#endif

#ifndef ROOT_ID
#	define ROOT_ID "ROOT"
#endif

namespace api {

struct Raw { // wrapper for arbitrary json data
	string data;
};

using Vector3 = array<double, 3>;
using Vector4 = array<double, 4>;
struct Axis: public Vector3 {}; // every Axis is a Vector3 but not every Vector3 is an Axis
const Axis xAxis {1, 0, 0};
const Axis yAxis {0, 1, 0};
const Axis zAxis {0, 0, 1};

enum class MatrixOrder { rowMajor, columnMajor };
struct Matrix33 {
	MatrixOrder order = MatrixOrder::rowMajor;
	array<array<double, 3>, 3> values = {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
};
struct Matrix44 {
	MatrixOrder order = MatrixOrder::rowMajor;
	array<array<double, 4>, 4> values = {{{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}};
};

enum class RefType { joint, trajectory };
struct Reference {
	RefType kind;
	string target;
};

template <bool allowReferences>
using Value = std::conditional_t<
	allowReferences,
	variant<double, Reference>,
	double
	>;

template <bool allowReferences>
using Value3 = array<Value<allowReferences>, 3>;

enum class QuatOrder { xyzw, wxyz };

struct Quaternion {
	QuatOrder order = QuatOrder::xyzw;
	Vector4 values = {0, 0, 0, 1};
};

enum class EulerOrder {
	z0x0z0, x0y0x0, y0z0y0, z0y0z0, x0z0x0, y0x0y0,
	x0y0z0, y0z0x0, z0x0y0, x0z0y0, z0y0x0, y0x0z0,
	z0x1z2, x0y1x2, y0z1y2, z0y1z2, x0z1x2, y0x1y2,
	x0y1z2, y0z1x2, z0x1y2, x0z1y2, z0y1x2, y0x1z2
};

template <bool allowReferences>
struct Euler {
	EulerOrder order;
	Value3<allowReferences> angles;
};

template <bool allowReferences>
struct AxisDistance {
	Axis axis;
	Value<allowReferences> distance;
};

template <bool allowReferences>
struct AxisAngle {
	Axis axis;
	Value<allowReferences> angle;
};

template <bool allowReferences>
struct DH {
	Value<allowReferences> a;
	Value<allowReferences> alpha;
	Value<allowReferences> theta;
	Value<allowReferences> s;
};

template <bool allowReferences>
using TransVec = Value3<allowReferences>;

using RotVec = Vector3;

template <bool allowReferences>
struct PosAndOri {
	variant<
		TransVec<allowReferences>,
		AxisDistance<allowReferences>
		> pos;
	variant<
		Quaternion,
		Matrix33,
		Euler<allowReferences>,
		AxisAngle<allowReferences>,
		RotVec
		> ori;
};

template <bool allowReferences>
using Transform = variant<
	PosAndOri<allowReferences>,
	Matrix44,
	DH<allowReferences>
	>;


enum class ShapeType { sphere, box, cylinder, orangeNet, mesh };

struct Sphere {
	Vector3 center = {0.0, 0.0, 0.0};
	double radius;
};

struct Box {
	Transform<false> pose;
	Vector3 size;
};

struct Cylinder {
	Vector3 center = {0.0, 0.0, 0.0};
	double radius;
	double height;
	Axis axis;
};

struct OrangeNet {
	vector<Vector3> points;
	double radius;
};

struct Mesh {
	vector<Vector3> vertices;
	vector<array<size_t, 3>> faces;
};

struct Shape {
	variant<Sphere, Box, Cylinder, OrangeNet, Mesh> geometry;
	bool collides = true;
	double margin = 0.03;
	double stickyForce = NaN;
};

struct Inertia {
	double ixx = 0.0;
	double iyy = 0.0;
	double izz = 0.0;
	double ixy = 0.0;
	double ixz = 0.0;
	double iyz = 0.0;
};

struct Link {
	string parent = ROOT_ID;
	Transform<true> pose;
	vector<string> parts;
};

struct RigidBody {
	double mass = 0.0;
	Vector3 centerOfMass = {0.0, 0.0, 0.0};
	Inertia inertia;
	Dict<Shape> shapes;
	Raw visuals;
};

struct Joint {
	double minValue = -inf;
	double maxValue = inf;
	double maxVelocity = inf;
	double maxAcceleration = inf;
	double maxDeceleration = inf;
	double maxJerk = inf;
	double maxEffort = inf;
};

enum class ModelType { robot, rigidBody };

struct RobotModel {
	Dict<Link> links;
	Dict<RigidBody> partModels;
	Dict<Joint> joints;
	Dict<vector<string>> collisionIgnoreGroups;
	Dict<Dict<double>> constraints;
	Raw visuals;
};

using Model = variant<RigidBody, RobotModel>;

struct Object {
	string model;
	string parent = WORLD_ID;
	Transform<true> pose;
	Dict<Value<true>> jointValues;
};

struct ControlPoint {
	double t = NaN;
	double f = NaN;
	double df = NaN;
	double ddf = NaN;
};

struct Trajectory {
	vector<ControlPoint> controlPoints;
	bool repeats = false;
};

struct Scene {
	array<int, 3> version;
	Dict<Model> models;
	Dict<Object> objects;
	Dict<vector<string>> collisionIgnoreGroups;
	Dict<Trajectory> trajectories;
};

} // namespace api
