
// A SweepingTransform describes a transformation (translation or rotation)
// along/around one axis. Given the axis and the limits of the transformation
// (distance/angle) (does not necessarily correspond to start and end), this
// class creates intermediate static transformation steps so that a swept convex
// hull can be copied and transformed to all the steps. Afterwards, a new convex
// hull must be created around all step convex hulls. This new convex hull is
// guaranteed to completely contain the original convex hull at an arbitrary
// distance/angle within the given limits (including in between steps).

#pragma once

#include "stl.h"
#include "transform.h"
#include "observer.h"

enum class SweepMode {
	unset,
	cartesianLinearSweep, // assumes linear Cartesian motion of every vertex of the convex hull (which is inaccurate)
	jointCuboidSweep // checks an axis-aligned cuboid in joint space (overconservative, guarantees full coverage)
};

struct SweepingValue {
	bool isActuated;
	double deflt;

	SweepMode mode = SweepMode::unset;

	// Be aware that in jointCuboidSweep mode, the limits represent minimum and maximum (not in order).
	// Both start and end can be somewhere in between.
	// In cartesianLinearSweep mode, the limits represent start and target.
	double limit1;
	double limit2;

	SweepingValue(bool isActuated, double deflt):
		isActuated {isActuated}, deflt {deflt}, limit1 {deflt}, limit2 {deflt} {}

	bool update(SweepMode mode, double limit1, double limit2) {
		if (this->mode == mode && limit1 == this->limit1 && limit2 == this->limit2) {
			return false;
		}
		else {
			this->mode = mode;
			this->limit1 = limit1;
			this->limit2 = limit2;
			return true;
		}
	}

	bool update(SweepMode mode, double value) {
		return update(mode, value, value);
	}
};

struct SweepingTransformConfig {
	Observable<SweepingValue>& observe;
	TransformType kind;
	Vector3 normalizedAxis;
	double maxRotationStep = 30.0_deg; // above 90 makes no sense, 180 crashes
};

class SweepingTransform {
	Observer<SweepingValue> value;
	vector<Transform> steps;
	SweepMode mode = SweepMode::unset;

public:

	const SweepingTransformConfig config;

	// moving or copying the object around would invalidate references in the update lambda capture
	SweepingTransform(const SweepingTransform&) = delete;
	SweepingTransform(SweepingTransform&&) = delete;

	SweepingTransform(SweepingTransformConfig config):
		config {config}
	{
		if (config.kind == TransformType::translation) {
			value.onUpdate([config, this](const SweepingValue& limits) {
				switch (limits.mode) {
					case SweepMode::unset:
						if (limits.limit1 != limits.limit2) {
							throw runtime_error("SweepingTransform: mode must be set if limits are not equal");
						}
					case SweepMode::cartesianLinearSweep: [[fallthrough]];
					case SweepMode::jointCuboidSweep:
						steps.clear();
						steps.push_back(axisDistanceToTransform(config.normalizedAxis, limits.limit1));
						if (limits.limit1 != limits.limit2) {
							steps.push_back(axisDistanceToTransform(config.normalizedAxis, limits.limit2));
						}
						break;
				}
				this->mode = limits.mode;
			});
		}
		else { // rotation
			value.onUpdate([config, this](const SweepingValue& limits) {
				if (limits.mode == SweepMode::unset && limits.limit1 != limits.limit2) {
					throw runtime_error("SweepingTransform: mode must be set if limits are not equal");
				}

				// note that the intermediate points need to be stretched out a bit
				// so that an arc from a to b around 0 is completely contained inside
				// the convex hull
				//                         a
				//                   __--''\ <- right angle
				//             __--''       \
				//       __--''             ⎞\
				//   0-==__          arc -> | X <- intermediate point
				//   |     ''--__           ⎠/
				//   |           ''--__     /
				// rotation axis       ''--/ <- right angle
				//                         b
				//
				// yes, drawing this took longer than coding it

				this->mode = limits.mode;

				const double& a = limits.limit1;
				const double& b = limits.limit2;

				if (a == b) {
					steps.clear();
					steps.push_back(axisAngleToTransform(config.normalizedAxis, a));
				}
				else {
					double delta = abs(b - a);
					size_t intermediateSteps = ceil(delta / config.maxRotationStep);
					double stepSize = delta / intermediateSteps;
					double stretch = 1.0 / cos(stepSize / 2.0);

					steps.clear();
					steps.reserve(2 + intermediateSteps * (limits.mode == SweepMode::jointCuboidSweep));
					steps.push_back(axisAngleToTransform(config.normalizedAxis, a));
					if (limits.mode == SweepMode::jointCuboidSweep) {
						for (size_t k = 0; k < intermediateSteps; k++) {
							steps.push_back(
								axisAngleToTransform(config.normalizedAxis, a + (k + 0.5) * stepSize)
								* planarScalingToTransform(config.normalizedAxis, stretch)
							);
						}
					}
					steps.push_back(axisAngleToTransform(config.normalizedAxis, b));
				}
			});
		}

		value.observe(config.observe);
	}

	bool isSweeping() const { return steps.size() > 1; }
	const vector<Transform>& getSteps() const { return steps; }
	SweepMode getMode() const { return mode; }
};
