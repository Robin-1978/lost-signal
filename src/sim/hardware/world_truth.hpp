#pragma once

#include "robot/autonomy/perception/dynamic_object.hpp"
#include "robot/platform/differential_kinematics.hpp"
#include "robot/runtime/observation.hpp"

#include <vector>

namespace sim {

struct WorldTruth {
 DiffState robot;
 std::vector<DynamicObjectState> dynamic_objects;
};

} // namespace sim
