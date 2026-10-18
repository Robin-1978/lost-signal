#include "tire_contact.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim::physics {

ResolvedTireContact resolve_tire_contact(const TireContactSpec& tire,
 const TireSurfaceContact& surface) {
 const double axle_length = std::hypot(
 std::hypot(tire.axle_axis_local.x, tire.axle_axis_local.y), tire.axle_axis_local.z);
 if (!(tire.base_friction >= 0.0) || !(tire.radius_m > 0.0) || axle_length < 1e-9 ||
 tire.spinning_resistance < 0.0 ||
 surface.friction_scale < 0.0 || surface.rolling_resistance < 0.0 ||
surface.contact_time_constant_s < 0.0 || surface.contact_damping_ratio < 0.0 ||
tire.contact_time_constant_s < 0.0 || tire.contact_damping_ratio < 0.0 ||
(surface.contact_time_constant_s > 0.0 && !(surface.contact_damping_ratio > 0.0)) ||
(tire.contact_time_constant_s > 0.0 && !(tire.contact_damping_ratio > 0.0)) ||
(surface.contact_time_constant_s > 0.0 && surface.contact_time_constant_s < 0.002) ||
(tire.contact_time_constant_s > 0.0 && tire.contact_time_constant_s < 0.002)) {
 throw std::invalid_argument("invalid tire/surface contact profile");
 }
 return {
 tire.base_friction * std::clamp(surface.friction_scale, 0.0, 2.0),
 surface.rolling_resistance * tire.radius_m,
 tire.spinning_resistance * tire.radius_m,
surface.contact_time_constant_s > 0.0
? surface.contact_time_constant_s
: tire.contact_time_constant_s,
surface.contact_time_constant_s > 0.0
? surface.contact_damping_ratio
: tire.contact_damping_ratio,
 };
}

} // namespace sim::physics
