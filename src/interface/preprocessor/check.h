
#pragma once

#include "stl.h"
#include "linalg.h"

void checkRealOrNan(double v) {
	if (isinf(v)) { throw runtime_error("real number or NaN expected"); }
}

void checkRealOrInf(double v) {
	if (isnan(v)) { throw runtime_error("real number or inf expected"); }
}

void checkReal(double v) {
	if (isnan(v) || isinf(v)) { throw runtime_error("real number expected"); }
}

void checkPositive(double v) {
	if (isnan(v) || v < 0) { throw runtime_error("positive number expected"); }
}

void checkPositiveOrZero(double v) {
	if (isnan(v) || v <= 0) { throw runtime_error("non-negative number expected"); }
}

template <size_t N>
void checkReal(const array<double, N>& a) {
	for (const auto& v: a) { checkReal(v); }
}

bool isNormalized(const api::Vector3& v) {
	return abs(dot(v, v) - 1) < EPS * EPS;
}

bool isOrthogonal(const api::Vector3& v1, const api::Vector3& v2) {
	return dot(v1, v2) < EPS;
}

void check(const api::Matrix33& matrix) {
	const auto& m = matrix.values;
	if (
		!isNormalized(m[0]) || !isNormalized(m[1]) || !isNormalized(m[2])
		|| isOrthogonal(m[0], m[1]) || isOrthogonal(m[1], m[2]) || isOrthogonal(m[0], m[2])
	) {
		throw runtime_error("not an orthonormal matrix");
	}
	if (dot(cross(m[0], m[1]), m[2]) < 0) {
		throw runtime_error("matrix flips handedness");
	}
}

void check(const api::Matrix44& matrix) {
	const auto& m = matrix.values;
	const api::Vector3 mx = {m[0][0], m[0][1], m[0][2]};
	const api::Vector3 my = {m[1][0], m[1][1], m[1][2]};
	const api::Vector3 mz = {m[2][0], m[2][1], m[2][2]};
	if (
		!isNormalized(mx) || !isNormalized(my) || !isNormalized(mz)
		|| isOrthogonal(mx, my) || isOrthogonal(my, mz) || isOrthogonal(mx, mz)
	) {
		throw runtime_error("rotation part is not an orthonormal matrix");
	}
	if (dot(cross(mx, my), mz) < 0) {
		throw runtime_error("rotation part flips handedness");
	}
	if (matrix.order == api::MatrixOrder::rowMajor) {
		if (m[3][0] != 0 || m[3][1] != 0 || m[3][2] != 0 || m[3][3] != 1) {
			throw runtime_error("last row of homogeneous transformation matrix must be (0, 0, 0, 1)");
		}
		checkReal(api::Vector3 {m[0][3], m[1][3], m[2][3]});
	}
	else {
		if (m[0][3] != 0 || m[1][3] != 0 || m[2][3] != 0 || m[3][3] != 1) {
			throw runtime_error("last row of homogeneous transformation matrix must be (0, 0, 0, 1)");
		}
		checkReal(api::Vector3 {m[3][0], m[3][1], m[3][2]});
	}
}


void check(const canonical::Inertia& inertia) {
	for (const auto& value: {
			 inertia.ixx, inertia.iyy, inertia.izz,
			 inertia.ixy, inertia.ixz, inertia.iyz
		 }) {
		checkPositiveOrZero(value);
	}
}

void check(const canonical::Sphere& sphere) {
	checkReal(sphere.center);
	checkPositiveOrZero(sphere.radius);
}

void check(const canonical::Box& box) {
	checkReal(box.size);
}

void check(const canonical::Shape& shape) {
	checkPositiveOrZero(shape.margin);
	if (!isnan(shape.stickyForce) && shape.stickyForce < 0) {
		throw runtime_error("sticky force must be NaN or >=0");
	}
	std::visit([](const auto& g) { return check(g); }, shape.geometry);
}

void check(const canonical::RigidBody& body) {
	checkPositiveOrZero(body.mass);
	checkReal(body.centerOfMass);
	check(body.inertia);
	for (const auto& [shapeId, shape]: body.shapes) {
		check(shape);
	}
}

void check(const canonical::Joint& joint) {
	if (joint.minValue > joint.maxValue) { throw runtime_error("joint min must be smaller than joint max"); }
	checkRealOrInf(joint.minValue);
	checkRealOrInf(joint.maxValue);
	checkPositive(joint.maxVelocity);
	checkPositive(joint.maxAcceleration);
	checkPositive(joint.maxDeceleration);
	checkPositive(joint.maxJerk);
	checkPositive(joint.maxEffort);
}

void check(const canonical::Model& model) {

	for (const auto& [linkId, link]: model.links) {
		for (const auto& partId: link.parts) {
			if (!model.partModels.count(partId)) {
				throw runtime_error("part model does not exist: "s
					+ partId);
			}
		}
		if (link.parent != ROOT_ID && !model.links.count(link.parent)) {
			throw runtime_error("link "s + linkId
				+ " has an invalid parent: " + link.parent);
		}
	}

	for (const auto& [jointId, joint]: model.joints) {
		check(joint);
	}

	for (const auto& [groupId, group]: model.collisionIgnoreGroups) {
		for (const auto& item: group) {
			if (!model.links.count(item)) {
				throw runtime_error("link in collision ignore group does not exist: "s
					+ groupId + "/" + item);
			}
		}
	}

	for (const auto& [constraintId, constraint]: model.constraints) {
		for (const auto& [jointId, coefficient]: constraint) {
			if (!model.joints.count(jointId)) {
				throw runtime_error("joint in constraint does not exist: "s
					+ constraintId + "/" + jointId);
			}
			checkReal(coefficient);
		}
	}
}
