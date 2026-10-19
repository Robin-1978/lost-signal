#pragma once

#include "sim/hardware/physics/physics_world.hpp"

namespace sim::physics {

// Analytic first hit against a closed, upright finite cylinder. Direction is
// normalized internally, so distance_m is always a metric range. The surface
// includes both circular caps as well as the curved side.
RaycastResult raycast_upright_finite_cylinder(
 const Ray& ray, BodyHandle body, Vec3 center_m, double radius_m, double height_m);

} // namespace sim::physics
