
// The Nodes in the Scene Graph

// A node can have many child nodes and one parent node, to which it is attached
// with a sweeping transformation along/around one axis (translation or
// rotation) or an arbitrary static transformation. Collision objects are also
// attached to nodes. Please check out the devlog entry about hierarchical
// collision checking.

#pragma once

#include "stl.h"
#include "bitmask.h"
#include "fileio.h"
#include "collisionobject.h"
#include "collision/collisionchecker.h"
#include "export/checkertojson.h"
#include "utils/profiling.h"

// todo: this must go into a separate file
using CollisionIgnoreGroup = unordered_set<CollisionObject*>;

class Node {
	Node* parent = nullptr;
	vector<Node*> children;
	Dict<unique_ptr<CollisionObject>> parts;
	Transform forwardKinematicsResult;

public:
	const string name = "";

	variant<Transform, SweepingTransform> trafo;

	Node(const string& name, const Transform& t):
		name {name}, trafo {t} {}

	Node(const string& name, const SweepingTransformConfig& cfg):
		name {name}, trafo(cfg) {}

	void appendChildNode(Node* child) {
		assert(child->parent == nullptr);
		child->parent = this;
		children.push_back(child);
	}

	const Node* getParent() const { return parent; }
	const vector<Node*>& getChildren() const { return children; }

	void appendPart(const string& name, const collision::CompoundShape& compound) {
		parts.insert({name, make_unique<CollisionObject>(this->name + "." + name, compound)});
	}

	CollisionObject* getPart(const string& name) {
		return parts.at(name).get();
	}

	const CollisionObject* getPart(const string& name) const {
		return parts.at(name).get();
	}

	Dict<unique_ptr<CollisionObject>>& getParts() {
		return parts;
	}

	const Dict<unique_ptr<CollisionObject>>& getParts() const {
		return parts;
	}

	void generateTemporaryCollisionBitMasks() {
		BitMask mask;
		mask.set(0);
		for (auto& [partName, part]: parts) {
			part->temporaryCollisionBitMask = mask;
		}
		size_t bit = 1;
		for (auto& child: children) {
			BitMask mask;
			mask.set(bit);
			child->assignTemporaryCollisionBitMask(mask);
			bit++;
		}
	}

	void assignTemporaryCollisionBitMask(BitMask mask) {
		for (auto& [partName, part]: parts) {
			part->temporaryCollisionBitMask = mask;
		}
		for (auto& child: children) {
			child->assignTemporaryCollisionBitMask(mask);
		}
	}

	bool isSweeping() {
		return std::holds_alternative<SweepingTransform>(trafo) &&
			get<SweepingTransform>(trafo).isSweeping();
	}

	void leftMultiplyTransform(const variant<Transform, SweepingTransform>& tf) {
		for (auto& child: children) {
			child->leftMultiplyTransform(tf);
		}
		for (auto& [partName, part]: parts) {
			part->leftMultiplyTransform(tf);
		}
	}

	void collectBulletCollisionObjects(
		vector<pair<btCollisionObject*, CollisionIgnoreGroupBitMasks>>& collection
	) {
		for (auto& [partName, part]: parts) {
			collection.push_back({
				part->makeTemporaryBulletObject(), {
					part->globalCollisionBitMask,
					part->temporaryCollisionBitMask
				}
			});
		}
		for (auto& child: children) {
			child->collectBulletCollisionObjects(collection);
		}
	}

	void computeForwardKinematics(Transform parentPose = Transform {}) {
		if (isSweeping()) { throw runtime_error("cannot compute forward kinematics when sweeping"); }
		if (holds_alternative<Transform>(trafo)) {
			forwardKinematicsResult = parentPose * std::get<Transform>(trafo);
		}
		else {
			forwardKinematicsResult = parentPose * std::get<SweepingTransform>(trafo).getSteps()[0];
		}
		for (const auto& child: children) {
			child->computeForwardKinematics(forwardKinematicsResult);
		}
	}

	const Transform& getForwardKinematicsResult() { return forwardKinematicsResult; }

	inline static size_t collisionDetectionCallCounter = 0;
	bool isSubTreeCollisionFree(
		const Dict<CollisionIgnoreGroup>& globalCollisionIgnoreGroups,
		int logLevel = 0,
		bool isRecursiveCall = false
	) {
		if (!isRecursiveCall) { collisionDetectionCallCounter++; }

		for (const auto& child: children) {
			bool subTreeCollisionFree = child->isSubTreeCollisionFree(
				globalCollisionIgnoreGroups, logLevel, true);
			if (not subTreeCollisionFree) {
				return false;
			}
		}
		leftMultiplyTransform(trafo);
		if (
			logLevel == 0 && (
				(children.size() == 1 && parts.size() == 0)
				|| children.size() == 0
				)
		) {
			return true;
		}

		generateTemporaryCollisionBitMasks();
		vector<pair<btCollisionObject*, CollisionIgnoreGroupBitMasks>> objects;
		collectBulletCollisionObjects(objects);

		CollisionChecker checker(objects);

		if (logLevel == 2 || (logLevel == 1 && !isRecursiveCall)) {
			PROFILE_RUN(debug export);

			json debugScene = checker;
			// writeFile("in_case_the_check_crashes.json", debugScene.dump(1, '\t'));

			auto report = checker.checkCollisions(true);

			json debugCollisions = report.collisions.value();
			json debugFile = {
				{"scene", debugScene},
				{"collisions", debugCollisions}
			};
			bool clear = report.numCollisions == 0;
			writeFile("../../log/scene" + zeroPad(collisionDetectionCallCounter, 4)
					+ "_" + name + (clear ? "_OK" : "_collision") + ".json",
				debugFile.dump(1, '\t'));

			PROFILE_PAUSE(debug export);

			return clear;
		}
		else {
			bool clear = checker.checkCollisions(false).numCollisions == 0;

			if (!clear && logLevel > 0) {
				PROFILE_RUN(debug export);

				json debugScene = checker;

				auto report = checker.checkCollisions(true);

				json debugCollisions = report.collisions.value();
				json debugFile = {
					{"scene", debugScene},
					{"collisions", debugCollisions}
				};

				writeFile("../../log/scene" + zeroPad(collisionDetectionCallCounter, 4)
						+ "_" + name + "_collision.json", debugFile.dump(1, '\t'));

				PROFILE_PAUSE(debug export);
			}
			return clear;
		}
	}

	void reset() {
		forwardKinematicsResult = Transform {};
		for (auto& child: children) {
			child->reset();
		}
		for (auto& [partName, part]: parts) {
			part->reset();
		}
	}

};
