#pragma once

#include <valarray>

template<typename T>
inline std::valarray<T> concat(const std::valarray<T>& va1, const std::valarray<T>& va2) {
	std::valarray<T> result(va1.size() + va2.size());

	for (size_t i = 0; i < va1.size(); i++) {
		result[i] = va1[i];
	}

	for (size_t i = 0; i < va2.size(); i++) {
		result[i + va1.size()] = va2[i];
	}

	return result;
}

template<typename T>
inline std::valarray<T> diff(const std::valarray<T>& y) {
	if (y.size() < 2) {
		return {};
	}
	std::valarray<T> result(y.size() - 1);
	for (size_t i = 0; i < y.size() - 1; i++) {
		result[i] = (y[i + 1] - y[i]);
	}
	return result;
}

inline std::valarray<std::valarray<double>> operator/ (
	const std::valarray<std::valarray<double>>& lhs,
	double val
	) {

	auto result = lhs;
	for (auto& v : result) {
		v /= val;
	}
	return result;
}

template<typename T>
inline T norm(const std::valarray<T>& va) {
	return sqrt((va * va).sum());
}

template<typename T>
inline T distance(const std::valarray<T>& a, const std::valarray<T>& b) {
	std::valarray<T> delta = b - a;
	return norm(delta);
}

template <typename T>
T sign(T val) {
	return (T(0) < val) - (val < T(0));
}

inline valarray<double> clampToCuboid(
	const valarray<double>& v,
	const valarray<double>& halfExtents,
	bool keepDirection
) {
	assert(v.size() == halfExtents.size());
	assert(halfExtents.min() > 0);
	if (keepDirection) {
		if (halfExtents.min() <= 0) {
			return v * 0;
		}
		double overshoot = abs(v / halfExtents).max();
		return v / overshoot;
	}
	else {
		auto result = v;
		for (size_t i = 0; i < result.size(); i++) {
			if (abs(result[i]) > halfExtents[i]) {
				result[i] = sign(result[i]) * halfExtents[i];
			}
		}
		return result;
	}
}

inline valarray<double> maximizeLengthWithinCuboid(
	const valarray<double>& v,
	const valarray<double>& halfExtents
) {
	assert(v.size() == halfExtents.size());
	return v / abs(v / halfExtents).max();
}

inline valarray<double> clampNorm(
	const valarray<double>& v,
	double limit
) {
	const double normSquared = (v * v).sum();
	if (normSquared <= limit * limit) {
		return v;
	}
	else {
		return v / sqrt(normSquared);
	}
}

inline double random(double to = 1) {
	return to * std::rand() / RAND_MAX;
}

inline double random(double from, double to) {
	return from + random(to - from);
}

inline std::valarray<double> randoms(size_t n, double to = 1) {
	std::valarray<double> result(0.0, n);
	for (auto& q : result) {
		q = random(to);
	}
	return result;
}

inline std::valarray<double> randoms(size_t n, double from, double to) {
	std::valarray<double> result(0.0, n);
	for (auto& q : result) {
		q = random(from, to);
	}
	return result;
}

template<typename S, typename T>
inline S& operator<<(S& os, const std::valarray<T>& va) {
	os << '[';
	string sep = "";
	for (const auto& v : va) {
		os << sep << v;
		sep = ", ";
	}
	os << ']';
	return os;
}

template<typename T>
inline string plot(const T& y){
	stringstream ss;
	ss << "import matplotlib.pyplot as plt\nplt.plot(" << y << ")\nplt.show()";
	return ss.str();
}
