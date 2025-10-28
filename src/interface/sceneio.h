#pragma once

#include "apiscene.h"

namespace api {

Scene loadScene(const std::string& file);
void saveScene(const std::string& file, const Scene& scene, bool pretty = false);

}
