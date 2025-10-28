
// As an InflatedConvexHullShape (OrangeNet) is transformed through space during
// the hierarchical collision detection, this class takes care that every
// sweeping transformation applied to it extrudes it further.

#pragma once

#include "sweepingtransform.h"
#include "collision/shapes/inflatedconvexhullshape.h"
#include "LinearMath/btTransform.h"
#include "utils/profiling.h"

class SweepingShape {

	unique_ptr<collision::InflatedConvexHullShape> shape;
	btTransform offset = btTransform::getIdentity();

public:
	class P2PSweep{}; // dispatch tag

	SweepingShape(const collision::InflatedConvexHullShape& templet) {
		shape = make_unique<collision::InflatedConvexHullShape>(templet.getRadius());
		shape->setSafetyMargin(templet.getSafetyMargin());
		for (size_t i = 0; i < templet.getNumPoints(); i++) {
			shape->addPoint(templet.getPoint(i));
		}
	}

	SweepingShape(
		P2PSweep,
		const collision::InflatedConvexHullShape& templet,
		const Transform& tfStart,
		const Transform& tfTarget
	) {
		shape = make_unique<collision::InflatedConvexHullShape>(templet.getRadius());
		shape->setSafetyMargin(templet.getSafetyMargin());

		for (size_t i = 0; i < templet.getNumPoints(); i++) {
			auto startPoint = tfStart * templet.getPoint(i);
			auto targetPoint = tfTarget * templet.getPoint(i);
			shape->addPoint(startPoint);
			shape->addPoint(targetPoint);
		}

		// PROFILE_RUN(convex hull optimization);
		// newShape->optimize();
		// PROFILE_PAUSE(convex hull optimization);
	}


	void leftMultiplyTransform(const btTransform& tf) {
		offset = tf * offset;
	}

	// if this SweepingTransform is actually sweeping, it will create a new shape
	// which invalidates any pointers to the old btCollisionShape inside
	// (so make sure to update every btCollisionObject pointing to this shape)
	void leftMultiplyTransform(const SweepingTransform& tf) {
		if (tf.getMode() == SweepMode::cartesianLinearSweep) {
			throw runtime_error("cartesianLinearSweep must be handled using p2pSweep");
		}
		if (not tf.isSweeping()) {
			leftMultiplyTransform(tf.getSteps().front());
		}
		else {
			auto newShape = make_unique<collision::InflatedConvexHullShape>(shape->getRadius());
			newShape->setSafetyMargin(shape->getSafetyMargin());

			for (size_t i = 0; i < shape->getNumPoints(); i++) {
				auto point = offset * shape->getPoint(i);
				for (const auto& stepTransform: tf.getSteps()) {
					newShape->addPoint(stepTransform * point);
				}
			}

			PROFILE_RUN(convex hull optimization);
			newShape->optimize();
			PROFILE_PAUSE(convex hull optimization);

			shape = std::move(newShape);
			offset.setIdentity();
		}
	}

	// not needed, dispatch happens further up
	// void leftMultiplyTransform(const variant<Transform, SweepingTransform>& tf) {
	// 	std::visit([this](auto&& tf_) { this->leftMultiplyTransform(tf_); }, tf);
	// }

	const btTransform& getOffset() const { return offset; }
	collision::InflatedConvexHullShape* getShape() { return shape.get(); }
	btConvexHullShape* getBulletShape() { return shape->getBulletShape(); }

};
