#pragma once

#include "physics_world.hpp"

#include <memory>
#include <string_view>

namespace sim::physics {

// Backend-neutral tire semantics. Crr-style spinning resistance is
// dimensionless; the finite-patch rolling support value is already a
// torque/load moment arm. A backend converts them to its native contact
// parameters. The axle is expressed in the tire body's local frame.
struct TireContactSpec {
 double base_friction{1.0};
 // The finite-patch rolling support moment prevents a point-contact cylinder
 // from toppling sideways and is distinct from longitudinal Crr.
 double rolling_support_moment_arm_m{0.0};
 double spinning_resistance{0.0};
 double radius_m{0.0};
 double normal_stiffness_Npm{0.0};
 double normal_damping_Nspm{0.0};
 Vec3 axle_axis_local;
 bool static_friction_anchor{true};
};

// Contact-pair values supplied by the active tire/surface profile. They are
// intentionally free of Bullet or Chrono types.
struct TireSurfaceContact {
 double friction_scale{1.0};
 double rolling_resistance{0.0};
 double normal_stiffness_Npm{0.0};
 double normal_damping_Nspm{0.0};
};

// Exposed for validation and telemetry. The length-valued resistance fields
// are backend parameters after unit conversion, not authored Crr values.
struct ResolvedTireContact {
 double friction{0.0};
 double rolling_friction_length_m{0.0};
 double spinning_friction_length_m{0.0};
 Vec3 rolling_anisotropy{1.0, 1.0, 1.0};
 double normal_stiffness_Npm{0.0};
 double normal_damping_Nspm{0.0};
};

// Maps the common tire contract into one physics backend. It configures the
// backend and reports resolved parameters; it never integrates contact force
// outside the backend's constraint solver.
class ITireContact {
 public:
 virtual ~ITireContact() = default;

 virtual std::string_view backend_name() const = 0;
 virtual void configure_body(BodySpec& body, const TireContactSpec& tire) const = 0;
 virtual ResolvedTireContact resolve(const TireContactSpec& tire,
 const TireSurfaceContact& surface) const = 0;
 virtual void apply(IPhysicsWorld& world, BodyHandle body,
 const TireContactSpec& tire, const TireSurfaceContact& surface) const = 0;
};

// Current production adapter. A future Chrono implementation belongs beside
// a complete Chrono dynamics provider so joints, bodies, tires and time are
// integrated by one solver.
std::unique_ptr<ITireContact> make_bullet_tire_contact();

} // namespace sim::physics
