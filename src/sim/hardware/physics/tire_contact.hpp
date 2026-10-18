#pragma once

#include "physics_world.hpp"

namespace sim::physics {

struct TireContactSpec {
 double base_friction{1.0};
 double spinning_resistance{0.0};
 double radius_m{0.0};
double contact_time_constant_s{0.0};
double contact_damping_ratio{0.0};
 Vec3 axle_axis_local;
};

struct TireSurfaceContact {
 double friction_scale{1.0};
 double rolling_resistance{0.0};
double contact_time_constant_s{0.0};
double contact_damping_ratio{0.0};
};

// MuJoCo's geom friction tuple contains tangential, torsional and rolling
// coefficients. The latter two are length-valued moment arms. Rolling loss
// comes from the dimensionless tire/surface Crr multiplied by wheel radius;
// an isotropic geom coefficient must not reuse a lateral support moment as it
// would also brake rotation about the wheel axle. Contact compliance remains
// in an engine-independent effective response profile until model compilation.
struct ResolvedTireContact {
 double friction{0.0};
 double rolling_friction_length_m{0.0};
 double spinning_friction_length_m{0.0};
double contact_time_constant_s{0.0};
double contact_damping_ratio{0.0};
};

ResolvedTireContact resolve_tire_contact(const TireContactSpec& tire,
 const TireSurfaceContact& surface);

} // namespace sim::physics
