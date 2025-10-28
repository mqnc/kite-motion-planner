
#pragma once

#include "stl.h"
#include "json.hpp"
#include "collision/shapes/shape.h"

using nlohmann::json;

inline void to_json(json& j, const btVector3& v) {
	j = json{ v[0], v[1], v[2] };
}

inline void to_json(json& j, const btQuaternion& q) {
	j = json{ q.x(), q.y(), q.z(), q.w() };
}

inline void to_json(json& j, const btTransform& tf) {
	j = json{
		{"xyz", tf.getOrigin()},
		{"qxyzw", tf.getRotation()}
	};
}

namespace collision{

void to_json(json& j, const Shape& s);

}