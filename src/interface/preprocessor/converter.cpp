
#include "converter.h"
#include "linalg.h"

string joinId(int i) { return std::to_string(i); }
string joinId(const string& s) { return s; }
template <typename... Ts>
string joinId(const string& first, Ts... rest) {
	return joinId(first) + "/" + joinId(rest...);
}

canonical::Reference toCanonical(const api::Reference& reference) {
	switch (reference.kind) {
		case api::RefType::joint: return canonical::Reference {canonical::RefType::joint, reference.target};
		case api::RefType::trajectory: return canonical::Reference {canonical::RefType::trajectory, reference.target};
	}
}

template <typename M>
canonical::TransformChain toCanonicalMatrix(const M& matrix) {
	// https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/

	std::cout << "matrix to quaternion conversion needs testing\n";

	double qw, qx, qy, qz;

	const auto& m = matrix.values;
	double trace = m[0][0] + m[1][1] + m[2][2];
	if (trace > 0) {
		double s = 0.5f / sqrtf(trace + 1.0f);
		qw = 0.25f / s;
		qx = (m[2][1] - m[1][2]) * s;
		qy = (m[0][2] - m[2][0]) * s;
		qz = (m[1][0] - m[0][1]) * s;
	} else {
		if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
			double s = 2.0f * sqrtf(1.0f + m[0][0] - m[1][1] - m[2][2]);
			qw = (m[2][1] - m[1][2]) / s;
			qx = 0.25f * s;
			qy = (m[0][1] + m[1][0]) / s;
			qz = (m[0][2] + m[2][0]) / s;
		} else if (m[1][1] > m[2][2]) {
			double s = 2.0f * sqrtf(1.0f + m[1][1] - m[0][0] - m[2][2]);
			qw = (m[0][2] - m[2][0]) / s;
			qx = (m[0][1] + m[1][0]) / s;
			qy = 0.25f * s;
			qz = (m[1][2] + m[2][1]) / s;
		} else {
			double s = 2.0f * sqrtf(1.0f + m[2][2] - m[0][0] - m[1][1]);
			qw = (m[1][0] - m[0][1]) / s;
			qx = (m[0][2] + m[2][0]) / s;
			qy = (m[1][2] + m[2][1]) / s;
			qz = 0.25f * s;
		}
	}

	auto q = (matrix.order == api::MatrixOrder::rowMajor) ?
		canonical::Quaternion {qx, qy, qz, qw}
		: canonical::Quaternion {-qx, -qy, -qz, qw};
	unsafeNormalize(q.xyzw);

	if constexpr (std::is_same_v<M, api::Matrix33>) {
		return {canonical::StaticTransform {{0, 0, 0}, q}};
	}
	else {
		static_assert(std::is_same_v<M, api::Matrix44>);
		if (matrix.order == api::MatrixOrder::rowMajor) {
			return {canonical::StaticTransform {{m[0][3], m[1][3], m[2][3]}, q}};
		}
		else {
			return {canonical::StaticTransform {{m[3][0], m[3][1], m[3][2]}, q}};
		}
	}
}

canonical::TransformChain toCanonical(const api::Matrix33& matrix) {
	return toCanonicalMatrix(matrix);
}
canonical::TransformChain toCanonical(const api::Matrix44& matrix) {
	return toCanonicalMatrix(matrix);
}


canonical::TransformChain toCanonical(api::Quaternion quaternion) {
	safeNormalize(quaternion.values);
	switch (quaternion.order) {
		case api::QuatOrder::xyzw: return {canonical::StaticTransform {
				{0, 0, 0},
				{quaternion.values[0], quaternion.values[1], quaternion.values[2], quaternion.values[3]}
			}};
		case api::QuatOrder::wxyz: return {canonical::StaticTransform {
				{0, 0, 0},
				{quaternion.values[1], quaternion.values[2], quaternion.values[3], quaternion.values[0]}
			}};
	}
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::AxisAngle<allowReferences>& axisAngle) {
	canonical::Axis axis = axisAngle.axis;
	safeNormalize(axis);
	double angle;
	if constexpr (allowReferences) {
		if (std::holds_alternative<api::Reference>(axisAngle.angle)) {
			return {canonical::DynamicTransform {
				canonical::TransformType::rotation,
				axis,
				toCanonical(std::get<api::Reference>(axisAngle.angle))
			}};
		}
		else {
			angle = std::get<double>(axisAngle.angle);
		}
	}
	else {
		angle = axisAngle.angle;
	}
	const double sinHalfAngle = sin(angle / 2);
	return {canonical::StaticTransform {
		{0, 0, 0},
		{axis[0] * sinHalfAngle, axis[1] * sinHalfAngle, axis[2] * sinHalfAngle, cos(angle / 2)}
	}};
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::AxisDistance<allowReferences>& axisDistance) {
	canonical::Axis axis = axisDistance.axis;
	safeNormalize(axis);
	double distance;
	if constexpr (allowReferences) {
		if (std::holds_alternative<api::Reference>(axisDistance.distance)) {
			return {canonical::DynamicTransform {
				canonical::TransformType::translation,
				axis,
				toCanonical(std::get<api::Reference>(axisDistance.distance))
			}};
		}
		else {
			distance = std::get<double>(axisDistance.distance);
		}
	}
	else {
		distance = axisDistance.distance;
	}
	return {canonical::StaticTransform {
		{axis[0] * distance, axis[1] * distance, axis[2] * distance},
		canonical::Quaternion {0, 0, 0, 1}
	}};
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::Euler<allowReferences>& euler) {

#define EULER_CASE(CASE, AXIS1, ANGLE1, AXIS2, ANGLE2, AXIS3, ANGLE3) \
	case api::EulerOrder::CASE: return { \
        toCanonical(api::AxisAngle<allowReferences> {api::AXIS1##Axis, euler.angles[ANGLE1]}).front(), \
        toCanonical(api::AxisAngle<allowReferences> {api::AXIS2##Axis, euler.angles[ANGLE2]}).front(), \
        toCanonical(api::AxisAngle<allowReferences> {api::AXIS3##Axis, euler.angles[ANGLE3]}).front() \
        };

		switch (euler.order) {
			EULER_CASE(z0x0z0, z, 2, x, 1, z, 0)
			EULER_CASE(x0y0x0, x, 2, y, 1, x, 0)
			EULER_CASE(y0z0y0, y, 2, z, 1, y, 0)
			EULER_CASE(z0y0z0, z, 2, y, 1, z, 0)
			EULER_CASE(x0z0x0, x, 2, z, 1, x, 0)
			EULER_CASE(y0x0y0, y, 2, x, 1, y, 0)
			EULER_CASE(x0y0z0, z, 2, y, 1, x, 0)
			EULER_CASE(y0z0x0, x, 2, z, 1, y, 0)
			EULER_CASE(z0x0y0, y, 2, x, 1, z, 0)
			EULER_CASE(x0z0y0, y, 2, z, 1, x, 0)
			EULER_CASE(z0y0x0, x, 2, y, 1, z, 0)
			EULER_CASE(y0x0z0, z, 2, x, 1, y, 0)
			EULER_CASE(z0x1z2, z, 0, x, 1, z, 2)
			EULER_CASE(x0y1x2, x, 0, y, 1, x, 2)
			EULER_CASE(y0z1y2, y, 0, z, 1, y, 2)
			EULER_CASE(z0y1z2, z, 0, y, 1, z, 2)
			EULER_CASE(x0z1x2, x, 0, z, 1, x, 2)
			EULER_CASE(y0x1y2, y, 0, x, 1, y, 2)
			EULER_CASE(x0y1z2, x, 0, y, 1, z, 2)
			EULER_CASE(y0z1x2, y, 0, z, 1, x, 2)
			EULER_CASE(z0x1y2, z, 0, x, 1, y, 2)
			EULER_CASE(x0z1y2, x, 0, z, 1, y, 2)
			EULER_CASE(z0y1x2, z, 0, y, 1, x, 2)
			EULER_CASE(y0x1z2, y, 0, x, 1, z, 2)
		}
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::DH<allowReferences>& dh) {
	array<api::Value<allowReferences>, 4> params = {dh.theta, dh.s, dh.a, dh.alpha};
	array<bool, 4> isRotation = {true, false, false, true};
	array<api::Axis, 4> axes = {api::zAxis, api::zAxis, api::xAxis, api::xAxis};

	canonical::TransformChain result;
	result.reserve(4);

	for (size_t i = 0; i < 4; i++) {
		if (isRotation[i]) {
			result.push_back(toCanonical(api::AxisAngle<allowReferences> {axes[i], params[i]}).front());
		}
		else {
			result.push_back(toCanonical(api::AxisDistance<allowReferences> {axes[i], params[i]}).front());
		}
	}

	return result;
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::TransVec<allowReferences>& translation) {
	const array<api::Axis, 3> axes = {api::xAxis, api::yAxis, api::zAxis};

	canonical::TransformChain result;
	result.reserve(4);

	for (size_t i = 0; i < 3; i++) {
		result.push_back(toCanonical(api::AxisDistance<allowReferences> {axes[i], translation[i]}).front());
	}

	return result;
}

canonical::TransformChain toCanonical(const api::RotVec& rotation) {
	const double angle = norm(rotation);

	if (angle < EPS) {
		return {canonical::StaticTransform {
			{0, 0, 0},
			{0, 0, 0, 1}
		}};
	}
	else {
		api::Axis axis = {rotation[0], rotation[1], rotation[2]};
		unsafeNormalize(axis);
		const double sinHalfAngle = sin(angle / 2);
		return {canonical::StaticTransform {
			{0, 0, 0},
			{axis[0] * sinHalfAngle, axis[1] * sinHalfAngle, axis[2] * sinHalfAngle, cos(angle / 2)}
		}};
	}
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::PosAndOri<allowReferences>& posAndOri) {
	auto result = std::visit( [](const auto& pos) {
			return toCanonical<allowReferences>(pos);
		}, posAndOri.pos);
	const auto rotationComponents = std::visit( [](const auto& ori) {
			return toCanonical(ori);
		}, posAndOri.ori);

	result.insert(result.end(), rotationComponents.begin(), rotationComponents.end());
	return result;
}

template <bool allowReferences>
canonical::TransformChain toCanonical(const api::Transform<allowReferences>& transform) {
	return std::visit( [](const auto& tf) {
			return toCanonical(tf);
		}, transform);
}

canonical::TransformChain combineStaticTransforms(const canonical::TransformChain& transforms) {
	canonical::TransformChain result;
	for (const auto& tf: transforms) {
		if (
			std::holds_alternative<canonical::StaticTransform>(tf)
			&& result.size() > 0
			&& std::holds_alternative<canonical::StaticTransform>(result.back())
		) {
			result.back() = std::get<canonical::StaticTransform>(result.back())
				* std::get<canonical::StaticTransform>(tf);
		}
		else {
			result.push_back(tf);
		}
	}
	return result;
}


canonical::Shape toCanonical(const api::Shape& shape) {
	using Geometry = decltype(canonical::Shape {}.geometry);

	canonical::Shape result {};

	result.collides = shape.collides;
	result.margin = shape.margin;
	result.stickyForce = shape.stickyForce;
	result.geometry = std::visit(
		overload {
			[](const api::Box& box) -> Geometry {
				auto trafos = combineStaticTransforms(toCanonical(box.pose));
				assert(trafos.size() == 1);
				return canonical::Box {
					std::get<canonical::StaticTransform>(trafos[0]),
					box.size
				};
			},
			[](const auto& other) -> Geometry {
				return other;
			}
		}, shape.geometry
	);

	return result;
}

canonical::RigidBody toCanonical(const api::RigidBody& body) {
	canonical::RigidBody result;
	result.mass = body.mass;
	result.centerOfMass = body.centerOfMass;
	result.inertia = body.inertia;
	result.visuals = body.visuals;
	for (const auto& [shapeId, shape]: body.shapes) {
		result.shapes.insert({shapeId, toCanonical(shape)});
	}
	return result;
}


canonical::Model toCanonicalModel(const api::RigidBody& body) {
	canonical::Model result;
	const auto [itPart, successPart] = result.partModels.insert({"PART", toCanonical(body)});
	result.partModels.insert({"PART", itPart->second});
	result.links.insert({"LINK", {
			ROOT_ID,
			{canonical::Transform{}},
			{"PART"}
		}});
	return result;
}

canonical::Model toCanonicalModel(const api::RobotModel& robot) {
	canonical::Model result;
	result.joints = robot.joints;
	for (const auto& [partModelId, partModel]: robot.partModels) {
		result.partModels.insert({partModelId, toCanonical(partModel)});
	}
	for (const auto& [linkId, linkTemplate]: robot.links) {
		result.links.insert({linkId, {
				linkTemplate.parent,
				toCanonical(linkTemplate.pose),
				linkTemplate.parts
			}});
	}
	result.constraints = robot.constraints;
	result.collisionIgnoreGroups = robot.collisionIgnoreGroups;
	result.visuals = robot.visuals;
	//check(result);

	return result;
}

canonical::Object toCanonical(const api::Object& object) {
	canonical::Object result;
	result.model = object.model;
	result.parent = object.parent;
	result.pose = toCanonical(object.pose);
	result.jointValues = object.jointValues;
	return result;
}

canonical::Scene preprocess(api::Scene scene) {
	canonical::Scene result;

	result.trajectories = scene.trajectories;

	for (const auto& [modelId, model]: scene.models) {
		result.models.insert({modelId, std::visit([](const auto& m) { return toCanonicalModel(m); }, model)});
	}

	for (const auto& [objectId, object]: scene.objects) {
		result.objects.insert({objectId, toCanonical(object)});
	}

	for (const auto& [groupId, group]: scene.collisionIgnoreGroups) {
		result.collisionIgnoreGroups.insert({groupId, group});
	}

	return result;
}
