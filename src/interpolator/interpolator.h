
#pragma once

#include "stl.h"
//#include "spline/spline.hpp"
#include "ruckig/ruckig.hpp"

class ContinuousTrajectory {
public:
	virtual double getDuration() const = 0;
	virtual valarray<double> at(double t) const = 0;

	virtual valarray<valarray<double>> sample(
		double dt, bool forceSampleAtTarget = false) const;

	virtual valarray<valarray<double>> sampleByDistance(
	 	double dtTest, double dDistanceApprox) const;

	virtual ~ContinuousTrajectory() = default;
};


class LinearPointToPointTrajectory :public ContinuousTrajectory {
	valarray<double> p0;
	valarray<double> p1;
	ruckig::Trajectory<1> progress;
public:
	LinearPointToPointTrajectory(
		const valarray<double>& p0,
		const valarray<double>& p1,
		const valarray<double>& maxVelocity,
		const valarray<double>& maxAcceleration,
		const valarray<double>& maxJerk
	);

	double getDuration() const override;

	valarray<double> at(double t) const override;
};


class OptimalTrajectory :public ContinuousTrajectory {
	ruckig::Trajectory<ruckig::DynamicDOFs> trajectory;
public:
	OptimalTrajectory(
		const valarray<double>& startPosition,
		const valarray<double>& startVelocity,
		const valarray<double>& startAcceleration,
		const valarray<double>& targetPosition,
		const valarray<double>& targetVelocity,
		const valarray<double>& targetAcceleration,
		const valarray<double>& maxVelocity,
		const valarray<double>& maxAcceleration,
		const valarray<double>& maxJerk
	);

	double getDuration() const override;

	valarray<double> at(double t) const override;
};


// class QuinticSplineTrajectory :public ContinuousTrajectory {
// 	Spline<valarray<double>> spline;
// 	double duration;
// public:
// 	QuinticSplineTrajectory(
// 		const valarray<double>& startPosition,
// 		const valarray<double>& startVelocity,
// 		const valarray<double>& startAcceleration,
// 		const valarray<double>& targetPosition,
// 		const valarray<double>& targetVelocity,
// 		const valarray<double>& targetAcceleration,
// 		double duration
// 	);

// 	double getDuration() const override;

// 	valarray<double> at(double t) const override;
// };


class CompoundTrajectory :public ContinuousTrajectory {
public:
	vector<unique_ptr<ContinuousTrajectory>> segments;

	double getDuration() const override;

	valarray<double> at(double t) const override;

	valarray<valarray<double>> sample(
		double dt,
		bool forceSampleAtTarget = false
	) const override;
};
