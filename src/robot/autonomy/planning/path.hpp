#pragma once

#include "robot/core/math.hpp"

#include <cstdint>

namespace sim {

// Direction used to arrive at a route point. The first point is normally
// Unspecified because no route segment precedes it.
enum class TravelDirection : std::int8_t {
 Unspecified = 0,
 Forward = 1,
 Reverse = -1,
};

// Canonical global-planning output. heading_rad is the nominal body yaw at
// position: a grid planner may derive it from the route tangent, while a
// kinematic planner preserves the yaw in its searched state. Position,
// heading, and gear stay together so no layer can accidentally discard
// car-like motion semantics.
struct RoutePoint {
 Vec2 position;
 double heading_rad{0.0};
 TravelDirection arrival_direction{TravelDirection::Unspecified};
};

} // namespace sim
