
// The Scene Graph

// holds all information about the collision world and stores everything that is
// referenced by pointers. This file mainly converts a canonical::Scene description
// to a data structure that the planner can operate on.

#pragma once

#include "stl.h"
#include "interface/preprocessor/canonicalscene.h"
#include "node.h"
#include "sweepingtransform.h"
#include "collision/shapes/inflatedconvexhullshape.h"
#include "collisionobject.h"

string qualify(int i) {
	return std::to_string(i);
}

string qualify(const string& s) {
	return s;
}

template <typename... Ts>
string qualify(const string& first, Ts... rest) {
	return qualify(first) + "." + qualify(rest...);
}

string qualifyReference(const string& nodeId, const canonical::Reference& ref) {
	string result;
	switch (ref.kind) {
		case canonical::RefType::joint: {
			string objectId = nodeId.substr(0, nodeId.find('.'));
			result = "joint:"s + qualify(objectId, ref.target);
			break;
		}
		case canonical::RefType::trajectory: {
			result = "trajectory:"s + ref.target;
			break;
		}
	}
	return result;
}

class SceneGraph {
public:
	Dict<Observable<SweepingValue>> referenceables;
	Dict<Node> nodes;
	Dict<collision::CompoundShape> partShapes;
	Dict<CollisionIgnoreGroup> collisionIgnoreGroups;

public:
	SceneGraph(const canonical::Scene& scene);

	void restoreDefaults();

	template <typename T>
	Node* addNode(const string& id, T&& param) {
		if (nodes.contains(id)) {
			throw runtime_error("multiple definitions of node "s + id);
		}
		auto [it, success] = nodes.try_emplace(id, id, std::forward<T>(param));
		return &(it->second);
	}

	Node* getFirstSubNode(const string& id) {
		return nodes.contains(qualify(id, 0)) ?
			&nodes.at(qualify(id, 0))
			: &nodes.at(id);
	}

	SweepingTransformConfig convertTransform(const canonical::DynamicTransform& trafo, const string& nodeId) {
		return SweepingTransformConfig {
			.observe = referenceables.at(qualifyReference(nodeId, trafo.value)),
			.kind = trafo.kind == canonical::TransformType::translation ?
				TransformType::translation
				: TransformType::rotation,
			.normalizedAxis = {trafo.axis[0], trafo.axis[1], trafo.axis[2]}
		};
	}

	static Transform convertTransform(const canonical::StaticTransform& trafo, const string& = "") {
		return posQuatToTransform(
			trafo.translation[0], trafo.translation[1], trafo.translation[2],
			trafo.rotation.xyzw[0], trafo.rotation.xyzw[1], trafo.rotation.xyzw[2], trafo.rotation.xyzw[3]
		);
	}

	void addNodeChainFromTrafos(const string& nodeId, canonical::TransformChain trafos) {
		for (size_t i = 0; i < trafos.size(); i++) {
			auto nodeId_i = i + 1 == trafos.size() ? nodeId : qualify(nodeId, std::to_string(i));
			const auto& trafo = trafos[i];

			std::visit([&](auto&& tf) {
				addNode(nodeId_i, this->convertTransform(tf, nodeId));
			}, trafo);
			if (i > 0) {
				nodes.at(qualify(nodeId, std::to_string(i - 1))).appendChildNode(&nodes.at(nodeId_i));
			}
		}
	};

	void assignGlobalCollisionBitMasks();

	bool isCollisionFree(int debugLevel) {
		bool result = nodes.at(WORLD_ID).isSubTreeCollisionFree(collisionIgnoreGroups, debugLevel);
		nodes.at(WORLD_ID).reset();
		return result;
	}

	void computeForwardKinematics() {
		nodes.at(WORLD_ID).computeForwardKinematics();
	}

	friend ostream& operator<<(ostream& os, const SceneGraph& sg);

};

ostream& operator<<(ostream& os, const SceneGraph& sg) {

	auto findNameOf = [&](const Node* n) {
		if (n == nullptr) { return "NULL"s; }
		for (const auto& [name, node]: sg.nodes) {
			if (&node == n) { return name; }
		}
		return "?"s;
	};

	os << "referenceables:\n\n";
	vector<string> names;
	for (auto& [name, _]: sg.referenceables) {
		names.push_back(name);
	}
	std::sort(names.begin(), names.end());
	for (const auto& name: names) { os << name << "\n"; }

	names.clear();
	os << "\nnodes:\n\n";
	for (auto& [name, node]: sg.nodes) {
		names.push_back(name + ", child of " + findNameOf(node.getParent()));
	}
	std::sort(names.begin(), names.end());
	for (const auto& name: names) { os << name << "\n"; }
	os << "\n";

	auto browse = [&](auto browse_, const Node* node, string indent = "") -> void {
		os << indent;
		os << "⸌-" << findNameOf(node);
		if (node->getParts().size() > 0) {
			os << "  [ ";
			for (const auto& [partName, part]: node->getParts()) {
				os << partName << " ";
			}
			os << "]";
		}
		os << "\n";
		for (const auto& child: node->getChildren()) {
			bool isLast = child == node->getChildren().back();
			browse_(browse_, child, indent + (isLast ? "    " : "   |"));
		}
	};
	browse(browse, &sg.nodes.at(WORLD_ID));

	os << "\ncollisionIgnoreGroups:\n\n";
	for (const auto& [groupName, group]: sg.collisionIgnoreGroups) {
		os << groupName << ":  [";
		for (const CollisionObject* object: group) {
			for (auto& [nodeName, node]: sg.nodes) {
				for (const auto& [partName, part]: node.getParts()) {
					if (object == part.get()) {
						os << " " << qualify(nodeName, partName) << " ";
					}
				}
			}
		}
		os << "]\n";
	}

	return os;
}

void SceneGraph::restoreDefaults() {
	for (auto& [id, ref]: referenceables) {
		ref.update(SweepMode::jointCuboidSweep, ref.getCached().deflt, ref.getCached().deflt);
	}
}

SceneGraph::SceneGraph(const canonical::Scene& scene) {

	// extract referenceable trajectories

	for (const auto& [name, trajectory]: scene.trajectories) {
		string fullName = "trajectory:"s + name;
		if (referenceables.contains(fullName)) {
			throw runtime_error("trajectory named "s + name + " defined multiple times");
		}
		referenceables.try_emplace(
			fullName,
			true,
			trajectory.controlPoints[0].f
		);
	}

	// extract referenceable joints

	for (const auto& [objectName, object]: scene.objects) {
		const auto& model = scene.models.at(object.model);

		for (const auto& [jointName, joint]: model.joints) {

			string fullName = "joint:"s + qualify(objectName, jointName);
			if (referenceables.contains(fullName)) {
				throw runtime_error("joint named "s + qualify(objectName, jointName) + " defined multiple times");
			}
			if(object.jointValues.count(jointName) == 0) {
				throw runtime_error("joint value "s + qualify(objectName, jointName) + " not defined");
			}
			if(not std::holds_alternative<double>(object.jointValues.at(jointName))){
				throw runtime_error("for planning, joint value "s + qualify(objectName, jointName) + " cannot be a reference");
			}
			referenceables.try_emplace(
				fullName,
				false,
				std::get<double>(object.jointValues.at(jointName))
			);
		}
	}

	restoreDefaults();

	// extract nodes from transformations

	addNode(WORLD_ID, Transform());
	for (const auto& [objectName, object]: scene.objects) {
		const auto& model = scene.models.at(object.model);

		addNodeChainFromTrafos(qualify(objectName, ROOT_ID), object.pose);

		for (const auto& [linkName, link]: model.links) {
			addNodeChainFromTrafos(qualify(objectName, linkName), link.pose);
		}
	}

	// hook up pointers, assemble tree structure

	for (const auto& [objectName, object]: scene.objects) {
		bool isQualified = (object.parent == WORLD_ID)
			|| (object.parent.find(".") != string::npos);
		Node& parent = nodes.at(isQualified ?
				object.parent
				: qualify(object.parent, ROOT_ID)
		);
		Node* root = getFirstSubNode(qualify(objectName, ROOT_ID));
		parent.appendChildNode(root);

		const auto& model = scene.models.at(object.model);

		for (const auto& [linkName, link]: model.links) {
			auto qualifiedName = qualify(objectName, linkName);
			Node& parent = nodes.at(qualify(objectName, link.parent));
			Node* child = getFirstSubNode(qualifiedName);
			parent.appendChildNode(child);
		}
	}

	// assemble parts

	for (const auto& [modelName, model]: scene.models) {
		for (const auto& [partName, part]: model.partModels) {

			auto inserted = partShapes.try_emplace(qualify(modelName, partName));
			collision::CompoundShape& compound = inserted.first->second;

			// object.setCollisionShape();

			// collision::CompoundShape

			for (const auto& [shapeName, shape]: part.shapes) {

				auto [trafo, collisionShape] =
					std::visit(
						overload {
							[](canonical::Sphere s) -> pair<Transform, unique_ptr<collision::Shape>> {
								auto result = make_pair(
									Transform(),
									make_unique<collision::InflatedConvexHullShape>(s.radius)
								);
								result.second->addPoint(s.center[0], s.center[1], s.center[2]);
								return result;
							},
							[](canonical::Box b) -> pair<Transform, unique_ptr<collision::Shape>> {
								return make_pair(
									SceneGraph::convertTransform(b.pose),
									make_unique<collision::BoxShape>(b.size[0], b.size[1], b.size[2])
								);
							},
							[](canonical::Cylinder c) -> pair<Transform, unique_ptr<collision::Shape>> {
								Vector3 center = {0.0, 0.0, 0.0};
								Vector3 z = Vector3::makeAxis(c.axis[0], c.axis[1], c.axis[2]);
								Vector3 helper = abs(z.x()) < abs(z.y()) ? xAxis : yAxis;
								Vector3 y = z.cross(helper).normalized();
								Vector3 x = y.cross(z);
								Vector3 t(c.center[0], c.center[1], c.center[2]);

								return make_pair(
									Transform({x, y, z}, t),
									make_unique<collision::CylinderZShape>(c.radius, c.height)
								);
							},
							[](canonical::OrangeNet o) -> pair<Transform, unique_ptr<collision::Shape>> {
								auto result = make_pair(
									Transform(),
									make_unique<collision::InflatedConvexHullShape>(o.radius)
								);
								for (const auto& p: o.points) {
									result.second->addPoint(p[0], p[1], p[2]);
								}
								return result;
							},
							[](canonical::Mesh) -> pair<Transform, unique_ptr<collision::Shape>> {
								throw runtime_error("todo");
							}
						}, shape.geometry
					);

				collisionShape->setSafetyMargin(shape.margin);
				compound.addShape(trafo, std::move(collisionShape));

				// shapes.insert({
				// 	qualify(modelName, partName, shapeName),

				// })
			}
		}
	}

	// attach parts to nodes

	for (const auto& [objectName, object]: scene.objects) {
		const auto& model = scene.models.at(object.model);
		for (const auto& [linkName, link]: model.links) {
			Node& node = nodes.at(qualify(objectName, linkName));
			for (const auto& partName: link.parts) {
				node.appendPart(partName, partShapes.at(qualify(object.model, partName)));
			}
		}
	}

	// fill collision ignore groups

	// todo: refactor all this:
	auto addPartsToGroup = [this]
		(
			auto& group,
			const string& objectName,
			const string& linkName,
			const string& partName = "*"
		) {
			if (partName != "*") {
				group.insert(nodes.at(qualify(objectName, linkName)).getPart(partName));
			}
			else {
				const auto& parts = this->nodes.at(qualify(objectName, linkName)).getParts();
				for (const auto& [partName, part]: parts) {
					group.insert(part.get());
				}
			}
		};

	for (const auto& [objectName, object]: scene.objects) {
		const auto& model = scene.models.at(object.model);
		for (const auto& [groupName, groupItems]: model.collisionIgnoreGroups) {
			auto inserted = collisionIgnoreGroups.try_emplace(qualify(objectName, groupName));
			CollisionIgnoreGroup& group = inserted.first->second;
			for (const auto& item: groupItems) {
				auto dotPos = item.find(".");
				bool isQualified = dotPos != string::npos;
				if (isQualified) {
					string linkName = item.substr(0, dotPos);
					string partName = item.substr(dotPos + 1);
					addPartsToGroup(group, objectName, linkName, partName);
				}
				else {
					const string& linkName = item;
					addPartsToGroup(group, objectName, linkName);
				}
			}
		}
	}

	for (const auto& [groupName, groupItems]: scene.collisionIgnoreGroups) {
		auto inserted = collisionIgnoreGroups.try_emplace(groupName);
		CollisionIgnoreGroup& group = inserted.first->second;
		for (const auto& item: groupItems) {
			auto dotPos1 = item.find(".");
			bool isQualified1 = dotPos1 != string::npos;
			if (isQualified1) {
				string objectName = item.substr(0, dotPos1);
				string objectItem = item.substr(dotPos1 + 1);
				auto dotPos2 = objectItem.find(".");
				bool isQualified2 = dotPos2 != string::npos;
				if (isQualified2) {
					string linkName = objectItem.substr(0, dotPos2);
					string partName = objectItem.substr(dotPos2 + 1);
					addPartsToGroup(group, objectName, linkName, partName);
				}
				else {
					const string& linkName = objectItem;
					addPartsToGroup(group, objectName, linkName);
				}
			}
			else {
				const string& objectName = item;
				const auto& object = scene.objects.at(objectName);
				const auto& model = scene.models.at(object.model);
				for (const auto& [linkName, link]: model.links) {
					addPartsToGroup(group, objectName, linkName);
				}
			}
		}
	}

	assignGlobalCollisionBitMasks();

	std::cout << *this << "\n";

}

void SceneGraph::assignGlobalCollisionBitMasks() {

	for (const auto& [nodeName, node]: nodes) {
		for (const auto& [partName, part]: node.getParts()) {
			part->globalCollisionBitMask.clear();
		}
	}

	size_t bit = 0;
	for (const auto& [groupName, group]: collisionIgnoreGroups) {
		for (const auto& item: group) {
			item->globalCollisionBitMask.set(bit);
		}
		bit++;
	}
}
