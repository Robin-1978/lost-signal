#pragma once

#include "autonomy/perception/dynamic_object.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "runtime/observation.hpp"

#include <vector>

namespace sim {

struct WorldTruth {
 DiffState robot;
 std::vector<DynamicObjectState> dynamic_objects;
};

} // namespace sim
