
// The aim here is to wrap bullet-specific transformations (Don'T Marry The
// Framework) and provide converters from all the ways a transformation can be
// specified in the api. We also initialize to sensible values.

#pragma once

#include "LinearMath/btTransform.h"

enum class TransformType { translation, rotation };

class Vector3: public btVector3 {
public:
	Vector3(): btVector3(0, 0, 0) {}

	// inherit constructors
	using btVector3::btVector3;

	// copy constructor is not inherited above :-/
	// https://stackoverflow.com/questions/57926023/why-an-inherited-constructor-is-not-a-candidate-for-initialization-from-an-expr // // // // // // // // //
	Vector3(const btVector3& other) {
		setValue(other[0], other[1], other[2]);
	}

	static Vector3 makeAxis(double x, double y, double z) {
		const double d2 = x * x + y * y + z * z;
		if (d2 < 1e-6) { throw runtime_error("cannot normalize zero vector"); }
		const double d = sqrt(d2);
		return Vector3(x / d, y / d, z / d);
	}
};

class Transform: public btTransform {
public:
	Transform(): btTransform(btTransform::getIdentity()) {}

	// inherit constructors
	using btTransform::btTransform;

	Transform(const btTransform& other) {
		setBasis(other.getBasis());
		setOrigin(other.getOrigin());
	}
};

const Vector3 xAxis(1, 0, 0);
const Vector3 yAxis(0, 1, 0);
const Vector3 zAxis(0, 0, 1);

inline Transform matrix33ToTransform(const array<array<double, 3>, 3>& values) {
	std::cout << "todo: sanitize" << "\n";
	return Transform(
		btMatrix3x3 {
			values[0][0], values[0][1], values[0][2],
			values[1][0], values[1][1], values[1][2],
			values[2][0], values[2][1], values[2][2]
		}
	);
}

inline Transform matrix44ToTransform(const array<array<double, 4>, 4>& values) {
	std::cout << "todo: sanitize" << "\n";
	return Transform(
		btMatrix3x3 {
			values[0][0], values[0][1], values[0][2],
			values[1][0], values[1][1], values[1][2],
			values[2][0], values[2][1], values[2][2]
		},
		btVector3 {
			values[0][3], values[1][3], values[2][3]
		}
	);
}

inline void sanitizeAxis(double& ex, double& ey, double& ez) {
	double norm = sqrt(ex * ex + ey * ey + ez * ez);
	if (norm < 0.001) {
		throw runtime_error("axis definition close to zero vector");
	}
	else {
		ex /= norm; ey /= norm; ez /= norm;
	}
}

inline void sanitizeQuaternion(double& qx, double& qy, double& qz, double& qw) {
	double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
	if (norm < 0.99 || norm > 1.01) {
		throw runtime_error("unnormalized quaternion");
	}
	else {
		qx /= norm; qy /= norm; qz /= norm; qw /= norm;
	}
}

inline Transform posToTransform(double x, double y, double z) {
	Transform result;
	result.setOrigin({x, y, z});
	return result;
}

inline Transform quatToTransform(double qx, double qy, double qz, double qw) {
	sanitizeQuaternion(qx, qy, qz, qw);
	return Transform(
		btQuaternion {qx, qy, qz, qw}
	);
}

inline Transform posQuatToTransform(double x, double y, double z, double qx, double qy, double qz, double qw) {
	sanitizeQuaternion(qx, qy, qz, qw);
	return Transform(
		btQuaternion {qx, qy, qz, qw},
		btVector3 {x, y, z}
	);
}

inline Transform axisDistanceToTransform(Vector3 axis, double distance) {
	sanitizeAxis(axis[0], axis[1], axis[2]);
	Transform result;
	result.setOrigin(axis * distance);
	return result;
}

inline Transform axisAngleToTransform(Vector3 axis, double angle) {
	sanitizeAxis(axis[0], axis[1], axis[2]);
	Transform result;
	result.setRotation(btQuaternion(axis, angle));
	return result;
}

// note that this is kind of abusing bullet which is only intended for translation and rotation
// it works for transforming individual points in convex hulls which is all we need
inline Transform planarScalingToTransform(const Vector3& normal, double scaling) {
	Transform result;
	double nx = normal[0];
	double ny = normal[1];
	double nz = normal[2];
	sanitizeAxis(nx, ny, nz);
	const double& s = scaling;

	result.setBasis(btMatrix3x3(
		s + (1 - s) * nx * nx, (1 - s) * ny * nx, (1 - s) * nz * nx,
		(1 - s) * nx * ny, s + (1 - s) * ny * ny, (1 - s) * nz * ny,
		(1 - s) * nx * nz, (1 - s) * ny * nz, s + (1 - s) * nz * nz
		));
	return result;
}

// class DynamicTransform: public Transform {
// protected:

// 	vector<double> _parameters;
// };

// class AxisAngle: public DynamicTransform {
// 	const btVector3 _axis;

// 	void _update() {
// 		btQuaternion rotation = btQuaternion(_axis, _parameters[0]);
// 		setRotation(rotation);
// 	}

// public:
// 	AxisAngle(const std::array<double, 3>& axis, double angle):
// 		_axis {axis[0], axis[1], axis[2]}
// 	{
// 		_parameters = {angle};
// 		_update();
// 	}

// };

// class DenavitHartenberg: public DynamicTransform {
// 	void _update() {
// 		const double a = _parameters[0];
// 		const double alpha = _parameters[1];
// 		const double theta = _parameters[2];
// 		const double s = _parameters[3];

// 		const double sinAlpha = sin(alpha);
// 		const double cosAlpha = cos(alpha);
// 		const double sinTheta = sin(theta);
// 		const double cosTheta = cos(theta);

// 		setBasis(btMatrix3x3(
// 			cosTheta, -sinTheta * cosAlpha, sinTheta * sinAlpha,
// 			sinTheta, cosTheta * cosAlpha, -cosTheta * sinAlpha,
// 			0, sinAlpha, cosAlpha
// 			));
// 		setOrigin(btVector3(
// 			a * cosTheta,
// 			a * sinTheta,
// 			s
// 			));
// 	}

// public:
// 	DenavitHartenberg(double a, double alpha, double theta, double s) {
// 		_parameters = {a, alpha, theta, s};
// 		_update();
// 	}

// };
