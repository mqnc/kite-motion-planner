#pragma once

#include "stl.h"
#include "bullet3/src/btBulletCollisionCommon.h"
#include "transform.h"
#include "collision/shapes/allshapes.h"
#include "sweepingshape.h"


class CollisionObject {

	btCollisionObject staticBulletObject;
	btCollisionObject temporarySweepingBulletObject;

	// flag whether we are already actually sweeping (sweeping transforms with no motion won't be counted)
	bool isSweeping = false;

	// keep track of the sweeping mode to prevent mixing different modes
	SweepMode sweepMode = SweepMode::unset;

	// tfStart and tfTarget are for cartesianLinearSweep:
	Transform tfStart;
	Transform tfTarget;

	collision::CompoundShape staticCompoundShape;
	collision::LinkingCompoundShape temporarySweepingCompoundShape;
	vector<SweepingShape> sweepingShapes;

	const collision::InflatedConvexHullShape& asHull(const collision::Shape* shape) const {
		const auto hull = dynamic_cast<const collision::InflatedConvexHullShape*>(shape);
		if (hull == nullptr) {
			throw runtime_error("only InflatedConvexHullShape can be swept");
		}
		return *hull;
	}

public:
	const string name = "";

	BitMask globalCollisionBitMask;
	BitMask temporaryCollisionBitMask;

	CollisionObject(const string& name, const collision::CompoundShape& compound):
		staticCompoundShape {compound}, name {name}
	{
		staticBulletObject.setCollisionShape(staticCompoundShape.getBulletShape());
		staticBulletObject.setUserPointer(this);
		staticBulletObject.setUserIndex(0);
	}

	void leftMultiplyTransform(const btTransform& tf) {
		tfStart = tf * tfStart;
		tfTarget = tf * tfTarget;

		if (not isSweeping) {
			staticBulletObject.setWorldTransform(
				tf * staticBulletObject.getWorldTransform()
			);
		}
		else {
			for (auto& shape: sweepingShapes) {
				shape.leftMultiplyTransform(tf);
			}
		}
	}

	void leftMultiplyTransform(const SweepingTransform& tf) {

		if (sweepMode == SweepMode::unset) { sweepMode = tf.getMode(); }
		if (sweepMode != tf.getMode()) { throw runtime_error("cannot mix different sweeping modes in a single collision query"); }

		if (not tf.isSweeping()) {
			leftMultiplyTransform(tf.getSteps().front());
			return;
		}

		if (tf.getMode() == SweepMode::cartesianLinearSweep) {
			tfStart = tf.getSteps().front() * tfStart;
			tfTarget = tf.getSteps().back() * tfTarget;
			const size_t n = staticCompoundShape.getNumShapes();
			sweepingShapes.clear();
			sweepingShapes.reserve(n);
			for (size_t i = 0; i < n; i++) {
				sweepingShapes.emplace_back(
					SweepingShape::P2PSweep {},
					asHull(staticCompoundShape.getShape(i)),
					tfStart,
					tfTarget
				);
			}
			isSweeping = true;
		}
		else if (tf.getMode() == SweepMode::jointCuboidSweep) {

			if (not isSweeping) {
				const size_t n = staticCompoundShape.getNumShapes();
				sweepingShapes.clear();
				sweepingShapes.reserve(n);
				for (size_t i = 0; i < n; i++) {
					sweepingShapes.emplace_back(asHull(staticCompoundShape.getShape(i)));
					sweepingShapes.back().leftMultiplyTransform(
						staticBulletObject.getWorldTransform() * staticCompoundShape.getShapeTrafo(i)
					);
				}
				isSweeping = true;
			}

			for (auto& shape: sweepingShapes) {
				shape.leftMultiplyTransform(tf);
			}
		}
		else {
			throw runtime_error("unhandled sweeping mode");
		}
	}

	void leftMultiplyTransform(const variant<Transform, SweepingTransform>& tf) {
		std::visit([this](auto&& tf_) { this->leftMultiplyTransform(tf_); }, tf);
	}

	btCollisionObject* makeTemporaryBulletObject() {
		// valid until the next leftMultiplyTransform call
		if (not isSweeping) {
			return &staticBulletObject;
		}
		else {
			temporarySweepingBulletObject = btCollisionObject();
			temporarySweepingBulletObject.setUserPointer(this);
			temporarySweepingBulletObject.setUserIndex(1);

			temporarySweepingCompoundShape.clear();
			for (auto& shape: sweepingShapes) {
				temporarySweepingCompoundShape.linkShape(shape.getOffset(), shape.getShape());
			}
			temporarySweepingBulletObject.setCollisionShape(temporarySweepingCompoundShape.getBulletShape());
			return &temporarySweepingBulletObject;
		}
	}

	void reset() {
		staticBulletObject.setWorldTransform(btTransform::getIdentity());
		temporarySweepingBulletObject.setCollisionShape(nullptr);
		sweepingShapes.clear();
		isSweeping = false;
		sweepMode = SweepMode::unset;
		tfStart = Transform();
		tfTarget = Transform();
	}

	static string btCollisionObjectToString(const btCollisionObject* obj) {
		if (obj->getUserPointer() == nullptr) { throw runtime_error("btCollisionObject is not member of a CollisionObject"); }
		auto collisionObject = static_cast<CollisionObject*>(obj->getUserPointer());
		if (obj->getUserIndex() == 0 && &(collisionObject->staticBulletObject) != obj) { throw runtime_error("invalid cast of btCollisionObject"); }
		else if (obj->getUserIndex() == 1 && &(collisionObject->temporarySweepingBulletObject) != obj) { throw runtime_error("invalid cast of btCollisionObject"); }
		string sweep = obj->getUserIndex() ? "(sweeping)" : "(static)";
		return collisionObject->name + sweep;
	}

};
