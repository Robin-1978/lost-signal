#pragma once

#include "core/math.hpp"

namespace sim {

// Estimated or simulated planar object state shared by sensing, perception,
// prediction, and local planning. Mission actor behavior stays outside the
// autonomy stack and produces this neutral observation type.
struct DynamicObjectState {
 Vec2 p{0.0, 0.0};
 Vec2 v{0.0, 0.0};
 double radius{0.45};
};

} // namespace sim
