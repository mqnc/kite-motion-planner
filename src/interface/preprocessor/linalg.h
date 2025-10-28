
#pragma once

#include "stl.h"
#include "../apiscene.h"
#include "canonicalscene.h"

const double EPS = 1e-7;

inline double dot(const api::Vector3& v1, const api::Vector3& v2) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline double dot(const api::Vector4& v1, const api::Vector4& v2) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2] + v1[3] * v2[3];
}

template <size_t N>
inline double norm(const array<double, N>& v) {
	return sqrt(dot(v, v));
}

inline array<double, 3> cross(const api::Vector3& v1, const api::Vector3& v2) {
	return {
		v1[1] * v2[2] - v1[2] * v2[1],
		v1[2] * v2[0] - v1[0] * v2[2],
		v1[0] * v2[1] - v1[1] * v2[0]
	};
}

template <size_t N>
void unsafeNormalize(array<double, N>& a) {
	double d = norm(a);
	for (size_t i = 0; i < N; i++) { a[i] /= d; }
}

template <size_t N>
void safeNormalize(array<double, N>& a) {
	double d = norm(a);
	if (d < EPS) { throw runtime_error("too close to 0 for normalization"); }
	for (size_t i = 0; i < N; i++) { a[i] /= d; }
}




canonical::Quaternion operator*(const canonical::Quaternion& quat1, const canonical::Quaternion& quat2) {
	const auto q1 = quat1.xyzw;
	const auto q2 = quat2.xyzw;
	const size_t x = 0, y = 1, z = 2, w = 3;
	// https://rosettacode.org/wiki/Quaternion_type#C++
	return {
		q1[w] * q2[x] + q1[x] * q2[w] + q1[y] * q2[z] - q1[z] * q2[y],
		q1[w] * q2[y] + q1[y] * q2[w] + q1[z] * q2[x] - q1[x] * q2[z],
		q1[w] * q2[z] + q1[z] * q2[w] + q1[x] * q2[y] - q1[y] * q2[x],
		q1[w] * q2[w] - q1[x] * q2[x] - q1[y] * q2[y] - q1[z] * q2[z]
	};
}

canonical::Quaternion operator*(const canonical::Quaternion& quat, const canonical::Vector3& vec) {
	const auto q2 = canonical::Quaternion {vec[0], vec[1], vec[2], 0};
	return quat * q2;
}

canonical::Quaternion conj(const canonical::Quaternion& quat) {
	const auto q = quat.xyzw;
	return {-q[0], -q[1], -q[2], q[3]};
}

canonical::StaticTransform operator*(const canonical::StaticTransform& t1, const canonical::StaticTransform& t2) {
	const auto temp = (t1.rotation * t2.translation * conj(t1.rotation)).xyzw;
	const canonical::Vector3 trans = { t1.translation[0] + temp[0], t1.translation[1] + temp[1], t1.translation[2] + temp[2]};
	const canonical::Quaternion rot = t1.rotation * t2.rotation;
	return {trans, rot};
}
