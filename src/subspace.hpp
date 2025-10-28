
#pragma once

#include "stl.h"
#include <Eigen/Dense>

inline Eigen::Map<Eigen::VectorXd> eigWrap(valarray<double>& v) {
	return Eigen::Map<Eigen::VectorXd>(&v[0], v.size());
}

inline const Eigen::Map<const Eigen::VectorXd> eigWrap(const valarray<double>& v) {
	return Eigen::Map<const Eigen::VectorXd>(&v[0], v.size());
}

// use Eigen::VectorXd ev = eigWrap(v) to copy
// use auto ev = eigWrap(v) to reference (and modify v)

class Subspace {
public:
	Eigen::MatrixXd eigConstraints;
	Eigen::VectorXd eigSupport;
	Eigen::MatrixXd nullSpace;
	std::mt19937 generator;
	valarray<double> lastRandomSuperSample;
	valarray<double> weights;

public:
	const size_t superDims = 0;
	const size_t subDims = 0;
	const valarray<array<double, 2>> superLimits;

	Subspace(
		const valarray<double>& support,
		const valarray<valarray<double>>& constraints,
		const valarray<array<double, 2>>& limits,
		std::random_device::result_type seed = std::random_device {}()
		):
		generator {seed},
		lastRandomSuperSample {support},
		superDims {support.size()},
		subDims {superDims - constraints.size()},
		superLimits {limits}
	{
		if (limits.size() != superDims) { throw runtime_error("limits size mismatch"); }
		if (subDims < 1) { throw runtime_error("too many constraints"); };
		if (constraints.size() == 0) { return; }

		eigConstraints = Eigen::MatrixXd(constraints.size(), superDims);

		for (size_t c = 0; c < constraints.size(); c++) {
			if (constraints[c].size() != superDims) {
				throw runtime_error("constraint size mismatch");
			}
			for (size_t j = 0; j < superDims; j++) {
				eigConstraints(c, j) = constraints[c][j];
			}
		}

		eigSupport = eigWrap(support);

		// the columns of the nullSpace matrix form the basis vectors of the space
		// in which we can move around without violating constraints
		nullSpace = Eigen::FullPivLU<Eigen::MatrixXd>(eigConstraints).kernel();

		// orthonormalize the nullspace basis
		nullSpace = nullSpace.householderQr().householderQ() * Eigen::MatrixXd::Identity(superDims, subDims);

	}

	valarray<double> project(
		const valarray<double>& params,
		bool isDerivative = false,
		double tolerance = 1e-6
	) {

		if (superDims == subDims) {
			return params;
		}

		const auto eigParams = eigWrap(params);
		if (
			(eigConstraints * (eigParams - eigSupport * !isDerivative))
				.cwiseAbs() .maxCoeff() > tolerance
		) {
			cout << "the following parameters do not meet the constraints: " << params << "\n";
			throw runtime_error("constraint discrepancy");
		}

		valarray<double> result(subDims);

		// only subtract support vector when we are not dealing with derivatives
		eigWrap(result) =
			Eigen::FullPivLU<Eigen::MatrixXd>(nullSpace).solve(
				eigParams - eigSupport * !isDerivative);

		return result;
	}

	valarray<valarray<double>> project(
		const valarray<valarray<double>>& trajectory,
		bool isDerivative = false,
		double tolerance = 1e-6
	) {
		if (superDims == subDims) {
			return trajectory;
		}

		valarray<valarray<double>> result(trajectory.size());
		for (size_t i = 0; i < trajectory.size(); i++) {
			result[i] = project(trajectory[i], isDerivative, tolerance);
		}
		return result;
	}

	valarray<double> unproject(
		const valarray<double>& params,
		bool isDerivative = false
	) {
		if (superDims == subDims) {
			return params;
		}

		const auto eigParams = eigWrap(params);

		valarray<double> result(superDims);

		// only add support vector when we are not dealing with derivatives
		eigWrap(result) = !isDerivative * eigSupport + nullSpace * eigParams;

		return result;
	}

	valarray<valarray<double>> unproject(
		const valarray<valarray<double>>& trajectory,
		bool isDerivative = false
	) {
		if (superDims == subDims) {
			return trajectory;
		}

		valarray<valarray<double>> result(trajectory.size());
		for (size_t i = 0; i < trajectory.size(); i++) {
			result[i] = unproject(trajectory[i], isDerivative);
		}
		return result;
	}

	valarray<double> randomSubSample(bool isDerivative = false, size_t iterations = 20) {

		if (superDims == subDims && !isDerivative) {
			valarray<double> result(superDims);
			for (size_t i = 0; i < superDims; i++) {
				std::uniform_real_distribution<double> distribution {superLimits[i][0], superLimits[i][1]};
				result[i] = distribution(generator);
			}
			return result;
		}

		if (isDerivative) {
			std::normal_distribution<double> distribution {0.0, 1.0};
			valarray<double> randomSubDirection(subDims);

			for (size_t i = 0; i < subDims; i++) {
				randomSubDirection[i] = distribution(generator);
			}

			double d = sqrt((randomSubDirection * randomSubDirection).sum());
			return randomSubDirection / d;
		}
		else {
			return project(randomSuperSample(false, iterations));
		}
	}

	valarray<double> randomSuperSample(bool isDerivative = false, size_t iterations = 20) {
		// https://mathoverflow.net/a/162327

		if (superDims == subDims) {
			return randomSubSample(isDerivative);
		}

		if (isDerivative) {
			return unproject(randomSubSample(true));
		}
		else {
			std::normal_distribution<double> distribution {0.0, 1.0};
			valarray<double> randomSubDirection(subDims);

			for (size_t i = 0; i < iterations; i++) {

				for (size_t i = 0; i < subDims; i++) {
					randomSubDirection[i] = distribution(generator);
				}

				valarray<double> randomSuperDirection = unproject(randomSubDirection, true);

				double minCoeff = -inf;
				double maxCoeff = inf;

				for (size_t i = 0; i < superDims; i++) {
					double lim1 = (superLimits[i][0] - lastRandomSuperSample[i]) / randomSuperDirection[i];
					double lim2 = (superLimits[i][1] - lastRandomSuperSample[i]) / randomSuperDirection[i];
					double minLim = min(lim1, lim2);
					double maxLim = max(lim1, lim2);
					minCoeff = max(minCoeff, minLim);
					maxCoeff = min(maxCoeff, maxLim);
				}

				std::uniform_real_distribution<double> distribution2 {minCoeff, maxCoeff};
				double coeff = distribution2(generator);

				lastRandomSuperSample = lastRandomSuperSample + coeff * randomSuperDirection;
			}
			return lastRandomSuperSample;
		}
	}
};
