
#include "sceneio.h"
#include "json.hpp"

using json = nlohmann::json;

namespace nlohmann {
// the usual to_json from_json definitions don't work with namespaces (like std)
// https://github.com/nlohmann/json/issues/1261#issuecomment-438448529
template <typename... Ts>
struct adl_serializer<std::variant<Ts...>> {
	static void to_json(json& j, const std::variant<Ts...>& v);
	static void from_json(const json& j, std::variant<Ts...>& v);
};
}

namespace api {

void to_json(json& j, const Raw& r) {
	j = r.data == "" ? json() : json::parse(r.data);
}

void from_json(const json& j, Raw& r) {
	r.data = j.dump();
	//s.visuals = j.value("visuals", json()).dump();
}

void to_json(json& j, const Axis& a) {
	if (a[0] == 1 && a[1] == 0 && a[2] == 0) { j = "x"; }
	else if (a[0] == 0 && a[1] == 1 && a[2] == 0) { j = "y"; }
	else if (a[0] == 0 && a[1] == 0 && a[2] == 1) { j = "z"; }
	else { j = json {a[0], a[1], a[2]}; }
}

void from_json(const json& j, Axis& a) {
	if (j.is_array()) {
		a[0] = j[0]; a[1] = j[1]; a[2] = j[2];
	}
	else if (j.is_string()) {
		string axis = j.get<string>();
		if (axis == "x") { a = {1, 0, 0}; }
		else if (axis == "y") { a = {0, 1, 0}; }
		else if (axis == "z") { a = {0, 0, 1}; }
		else { throw runtime_error("unsupported axis definition: "s + axis); }
	}
	else {
		throw runtime_error("expected array or string, not "s + j.dump());
	}
}


void to_json(json& j, const Reference& r) {
	switch (r.kind) {
		// case RefType::file: j = {{"file", r.target}}; break;
		case RefType::joint: j = {{"joint", r.target}}; break;
		case RefType::trajectory: j = {{"trajectory", r.target}}; break;
	}
}

void from_json(const json& j, Reference& r) {
	// if (j.contains("file")) { r = Reference {RefType::file, j.at("file")}; return; }
	if (j.contains("joint")) { r = Reference {RefType::joint, j.at("joint")}; return; }
	if (j.contains("trajectory")) {
		r = Reference {
			RefType::trajectory,
			j.at("trajectory")
		};
		if (j.contains("index")) {
			r.target += "." + to_string(j.at("index").get<size_t>());
		}
		return;
	}
}

void to_json(json& j, const Value<true>& v) {
	std::visit([&j](auto&& valOrRef) { j = valOrRef; }, v);
}

void from_json(const json& j, Value<true>& v) {
	if (j.is_number()) { v = j.get<double>(); }
	else if (j.is_object()) { v = j.get<Reference>(); }
	else { throw runtime_error("expected number or object, not "s + j.dump()); }
}

NLOHMANN_JSON_SERIALIZE_ENUM(QuatOrder, {
		{QuatOrder::xyzw, "xyzw"},
		{QuatOrder::wxyz, "wxyz"}
	});

NLOHMANN_JSON_SERIALIZE_ENUM(EulerOrder, {
		{EulerOrder::z0x0z0, "z0x0z0"}, {EulerOrder::x0y0x0, "x0y0x0"},
		{EulerOrder::y0z0y0, "y0z0y0"}, {EulerOrder::z0y0z0, "z0y0z0"},
		{EulerOrder::x0z0x0, "x0z0x0"}, {EulerOrder::y0x0y0, "y0x0y0"},
		{EulerOrder::x0y0z0, "x0y0z0"}, {EulerOrder::y0z0x0, "y0z0x0"},
		{EulerOrder::z0x0y0, "z0x0y0"}, {EulerOrder::x0z0y0, "x0z0y0"},
		{EulerOrder::z0y0x0, "z0y0x0"}, {EulerOrder::y0x0z0, "y0x0z0"},
		{EulerOrder::z0x1z2, "z0x1z2"}, {EulerOrder::x0y1x2, "x0y1x2"},
		{EulerOrder::y0z1y2, "y0z1y2"}, {EulerOrder::z0y1z2, "z0y1z2"},
		{EulerOrder::x0z1x2, "x0z1x2"}, {EulerOrder::y0x1y2, "y0x1y2"},
		{EulerOrder::x0y1z2, "x0y1z2"}, {EulerOrder::y0z1x2, "y0z1x2"},
		{EulerOrder::z0x1y2, "z0x1y2"}, {EulerOrder::x0z1y2, "x0z1y2"},
		{EulerOrder::z0y1x2, "z0y1x2"}, {EulerOrder::y0x1z2, "y0x1z2"}
	});

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Euler<true>,
	order, angles);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Euler<false>,
	order, angles);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AxisDistance<true>,
	axis, distance);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AxisDistance<false>,
	axis, distance);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AxisAngle<true>,
	axis, angle);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(AxisAngle<false>,
	axis, angle);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DH<true>,
	a, alpha, theta, s);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DH<false>,
	a, alpha, theta, s);

template <bool allowReferences>
void to_json(json& j, const Transform<allowReferences>& t) {

	std::visit(overload
		{
			[&j](const PosAndOri<allowReferences>& po) {
				j = json::object();
				std::visit(overload
					{
						[&j](const Value3<allowReferences>& ar) { j["xyz"] = ar; },
						[&j](const AxisDistance<allowReferences>& ad) { j["axisDistance"] = ad; }
					}, po.pos
				);
				std::visit(overload
					{
						[&j](const Quaternion& q) {
							j[q.order == QuatOrder::xyzw ? "qxyzw" : "qwxyz"] = q.values;
						},
						[&j](const Matrix33& m) { j["rows3x3"] = m.values; }, // todo: consider order
						[&j](const Euler<allowReferences>& e) { j["euler"] = e; },
						[&j](const AxisAngle<allowReferences>& aa) { j["axisAngle"] = aa; },
						[&j](const RotVec& ar) { j["rotvec"] = ar; }
					}, po.ori
				);
			},
			[&j](const Matrix44& m) {
				if (m.order == MatrixOrder::rowMajor) {
					j = {{"rows4x4", m.values}};
				} else {
					j = {{"cols4x4", m.values}};
				}
			},
			[&j](const DH<allowReferences>& dh) {
				j = {{"dh", dh}};
			},
		}, t
	);
}

template <bool allowReferences>
void from_json(const json& j, Transform<allowReferences>& t) {
	if (j.contains("cols4x4")) {
		auto m = j.at("cols4x4");
		t = Matrix44 {
			MatrixOrder::columnMajor,
			{{
				{m.at(0).at(0), m.at(0).at(1), m.at(0).at(2), m.at(0).at(3)},
				{m.at(1).at(0), m.at(1).at(1), m.at(1).at(2), m.at(1).at(3)},
				{m.at(2).at(0), m.at(2).at(1), m.at(2).at(2), m.at(2).at(3)},
				{m.at(3).at(0), m.at(3).at(1), m.at(3).at(2), m.at(3).at(3)}
			}}
		};
		return;
	}
	if (j.contains("rows4x4")) {
		auto m = j.at("rows4x4");
		t = Matrix44 {
			MatrixOrder::rowMajor,
			{{
				{m.at(0).at(0), m.at(0).at(1), m.at(0).at(2), m.at(0).at(3)},
				{m.at(1).at(0), m.at(1).at(1), m.at(1).at(2), m.at(1).at(3)},
				{m.at(2).at(0), m.at(2).at(1), m.at(2).at(2), m.at(2).at(3)},
				{m.at(3).at(0), m.at(3).at(1), m.at(3).at(2), m.at(3).at(3)}
			}}
		};
		return;
	}
	if (j.contains("dh")) { t = DH<allowReferences> {
			.a = j.at("dh").at("a"),
			.alpha = j.at("dh").at("alpha"),
			.theta = j.at("dh").at("theta"),
			.s = j.at("dh").at("s")
		}; return; }

	PosAndOri<allowReferences> result;
	if (j.contains("xyz")) {
		result.pos = j.at("xyz").get<Value3<allowReferences>>();
	}
	else if (j.contains("axisDistance")) {
		result.pos = j.at("axisDistance").get<AxisDistance<allowReferences>>();
	}

	if (j.contains("qwxyz")) {
		result.ori = Quaternion {QuatOrder::wxyz, j.at("qwxyz").get<Vector4>()};
	}
	else if (j.contains("qxyzw")) {
		result.ori = Quaternion {QuatOrder::xyzw, j.at("qxyzw").get<Vector4>()};
	}
	else if (j.contains("euler")) {
		result.ori = j.at("euler").get<Euler<allowReferences>>();
	}
	else if (j.contains("axisAngle")) {
		result.ori = j.at("axisAngle").get<AxisAngle<allowReferences>>();
	}
	else if (j.contains("rotvec")) {
		result.ori = j.at("rotvec").get<RotVec>();
	}

	if (j.contains("cols3x3") || j.contains("rows3x3")) {
		t = Transform<allowReferences>();
		throw runtime_error("todo");
	}
	t = result;
}

NLOHMANN_JSON_SERIALIZE_ENUM(ShapeType, {
		{ShapeType::sphere, "sphere"},
		{ShapeType::box, "box"},
		{ShapeType::cylinder, "cylinder"},
		{ShapeType::orangeNet, "orangeNet"},
		{ShapeType::mesh, "mesh"}
	});

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Sphere,
	center, radius);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Box,
	pose, size);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Cylinder,
	center, radius, height, axis);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(OrangeNet,
	points, radius);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Mesh,
	vertices, faces);

void to_json(json& j, const Shape& s) {

	std::visit([&j](auto&& geom) { j = geom; }, s.geometry);

	std::visit(overload
		{
			[&j](const Sphere&) { j["kind"] = ShapeType::sphere; },
			[&j](const Box&) { j["kind"] = ShapeType::box; },
			[&j](const Cylinder&) { j["kind"] = ShapeType::cylinder; },
			[&j](const OrangeNet&) { j["kind"] = ShapeType::orangeNet; },
			[&j](const Mesh&) { j["kind"] = ShapeType::mesh; }
		}, s.geometry
	);

	j["collides"] = s.collides;
	j["margin"] = s.margin;
	if (!isnan(s.stickyForce)) { j["stickyForce"] = s.stickyForce; }
}

void from_json(const json& j, Shape& s) {
	Shape defaults;
	s.collides = j.value("collides", defaults.collides);
	s.margin = j.value("margin", defaults.margin);
	s.stickyForce = j.value("stickyForce", defaults.stickyForce);

	switch (j.at("kind").get<ShapeType>()) {
		case ShapeType::sphere: s.geometry = j.get<Sphere>(); break;
		case ShapeType::box: s.geometry = j.get<Box>(); break;
		case ShapeType::cylinder: s.geometry = j.get<Cylinder>(); break;
		case ShapeType::orangeNet: s.geometry = j.get<OrangeNet>(); break;
		case ShapeType::mesh: s.geometry = j.get<Mesh>(); break;
	};
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Inertia,
	ixx, iyy, izz, ixy, ixz, iyz);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(RigidBody,
	mass, centerOfMass, inertia, shapes, visuals);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Link,
	parent, pose, parts);

struct JointHelper : public Joint {
    JointHelper() = default;
    JointHelper(const Joint& j) : Joint(j) {}
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(JointHelper,
	minValue, maxValue,
	maxVelocity, maxAcceleration, maxDeceleration,
	maxJerk, maxEffort);

void to_json(json& j, const Joint& joint) {
	j = json(static_cast<JointHelper>(joint));
	if (isnan(joint.minValue)) { j.erase("minValue"); }
	if (isnan(joint.maxValue)) { j.erase("maxValue"); }
	if (isnan(joint.maxVelocity)) { j.erase("maxVelocity"); }
	if (isnan(joint.maxAcceleration)) { j.erase("maxAcceleration"); }
	if (isnan(joint.maxDeceleration)) { j.erase("maxDeceleration"); }
	if (isnan(joint.maxJerk)) { j.erase("maxJerk"); }
	if (isnan(joint.maxEffort)) { j.erase("maxEffort"); }
}

void from_json(const json& j, Joint& joint) {
	joint = j.get<JointHelper>();
}

void to_json(json& j, const ControlPoint& p) {
	j = json {};
	j["t"] = p.t;
	j["f"] = p.f;
	if (!isnan(p.df)) { j["df"] = p.df; }
	if (!isnan(p.ddf)) { j["ddf"] = p.ddf; }
}

void to_json(json& j, const Trajectory& t) {
	j = {
		{"controlPoints", t.controlPoints},
		{"repeats", t.repeats}
	};
}

void from_json(const json& j, Dict<Trajectory>& ts) {
	for (auto& [key, jt]: j.items()) {
		const auto& jcps = jt.at("controlPoints");
		const auto& jcp0 = jcps.at(0);
		int nDims = -1;
		if (jcp0.at("f").is_array()) { nDims = jcp0.at("f").size(); }

		if (nDims == -1) {
			const auto [it, success] = ts.insert({key, {}});
			auto& trajectory = it->second;
			if (jt.contains("repeats")) { trajectory.repeats = jt.at("repeats"); }
			for (const auto& jcp: jcps) {
				trajectory.controlPoints.push_back({});
				auto& cp = trajectory.controlPoints.back();
				if (jcp.contains("t")) { jcp.at("t").get_to(cp.t); }
				jcp.at("f").get_to(cp.f);
				if (jcp.contains("df")) { jcp.at("df").get_to(cp.df); }
				if (jcp.contains("ddf")) { jcp.at("ddf").get_to(cp.ddf); }
			}
		}
		else {
			for (int i = 0; i < nDims; i++) {
				const auto [it, success] = ts.insert({key + "." + to_string(i), {}});
				auto& trajectory = it->second;
				if (jt.contains("repeats")) { trajectory.repeats = jt.at("repeats"); }
				for (const auto& jcp: jcps) {
					trajectory.controlPoints.push_back({});
					auto& cp = trajectory.controlPoints.back();
					if (jcp.contains("t")) { jcp.at("t").get_to(cp.t); }
					jcp.at("f").at(i).get_to(cp.f);
					if (jcp.contains("df")) { jcp.at("df").at(i).get_to(cp.df); }
					if (jcp.contains("ddf")) { jcp.at("ddf").at(i).get_to(cp.ddf); }
				}
			}
		}
	}
}

NLOHMANN_JSON_SERIALIZE_ENUM(ModelType, {
		{ModelType::robot, "robot"},
		{ModelType::rigidBody, "rigidBody"}
	});

void to_json(json& j, const Model& m) {
	std::visit( [&](auto&& model) {
			using T = std::decay_t<decltype(model)>;
			if constexpr (std::is_same_v<T, RobotModel>) {
				j = {
					{"kind", ModelType::robot},
					{"links", model.links},
					{"partModels", model.partModels},
					{"joints", model.joints},
					{"collisionIgnoreGroups", model.collisionIgnoreGroups},
					{"constraints", model.constraints},
					{"visuals", model.visuals}
				};
			} else if constexpr (std::is_same_v<T, RigidBody>) {
				j = model;
				j["kind"] = ModelType::rigidBody;
			}
		}, m);
}

void from_json(const json& j, Model& m) {
	auto kind = j.at("kind").get<ModelType>();
	if (kind == ModelType::robot) {
		RobotModel robotModel;
		j.at("links").get_to(robotModel.links);
		j.at("partModels").get_to(robotModel.partModels);
		j.at("joints").get_to(robotModel.joints);
		j.at("collisionIgnoreGroups").get_to(robotModel.collisionIgnoreGroups);
		j.at("constraints").get_to(robotModel.constraints);
		j.at("visuals").get_to(robotModel.visuals);
		m = robotModel;
	} else if (kind == ModelType::rigidBody) {
		m = j.get<RigidBody>();
	}
}

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Object,
	model, parent, pose, jointValues);

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Scene,
	models, objects, collisionIgnoreGroups, trajectories);

Scene loadScene(const std::string& file) {
	std::ifstream f(file);
	json data = json::parse(f);
	return data.get<Scene>();
}

void saveScene(const std::string& file, const Scene& scene, bool pretty) {
	std::ofstream f(file);
	json data = scene;
	if (pretty) { f << data.dump(1, '\t'); }
	else { f << data.dump(); }
}

} // namespace api

namespace nlohmann {
template <typename... Ts>
void adl_serializer<std::variant<Ts...>>::to_json(json& j, const std::variant<Ts...>& v) { api::to_json(j, v); }
template <typename... Ts>
void adl_serializer<std::variant<Ts...>>::from_json(const json& j, std::variant<Ts...>& v) { api::from_json(j, v); }
}
