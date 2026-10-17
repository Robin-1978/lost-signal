#include "tire_contact.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim::physics {
namespace {

double length(Vec3 value) {
 return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

void validate(const TireContactSpec& tire) {
 if (!std::isfinite(tire.base_friction) || tire.base_friction < 0.0 ||
 !std::isfinite(tire.rolling_support_moment_arm_m) ||
 tire.rolling_support_moment_arm_m < 0.0 ||
 !std::isfinite(tire.spinning_resistance) || tire.spinning_resistance < 0.0 ||
 !std::isfinite(tire.radius_m) || !(tire.radius_m > 0.0) ||
 !std::isfinite(tire.normal_stiffness_Npm) || tire.normal_stiffness_Npm < 0.0 ||
 !std::isfinite(tire.normal_damping_Nspm) || tire.normal_damping_Nspm < 0.0 ||
 !std::isfinite(tire.axle_axis_local.x) || !std::isfinite(tire.axle_axis_local.y) ||
 !std::isfinite(tire.axle_axis_local.z) || length(tire.axle_axis_local) < 0.5) {
 throw std::invalid_argument("invalid tire contact specification");
 }
}

void validate(const TireSurfaceContact& surface) {
 if (!std::isfinite(surface.friction_scale) || surface.friction_scale < 0.0 ||
 !std::isfinite(surface.rolling_resistance) || surface.rolling_resistance < 0.0 ||
 !std::isfinite(surface.normal_stiffness_Npm) || !(surface.normal_stiffness_Npm > 0.0) ||
 !std::isfinite(surface.normal_damping_Nspm) || surface.normal_damping_Nspm < 0.0) {
 throw std::invalid_argument("invalid tire-surface contact specification");
 }
}

class BulletTireContact final : public ITireContact {
 public:
 std::string_view backend_name() const override { return "bullet"; }

 void configure_body(BodySpec& body, const TireContactSpec& tire) const override {
 validate(tire);
 body.friction = tire.base_friction;
 body.rolling_friction = tire.rolling_support_moment_arm_m;
 body.spinning_friction = tire.spinning_resistance * tire.radius_m;
 body.contact_stiffness_Npm = tire.normal_stiffness_Npm;
 body.contact_damping_Nspm = tire.normal_damping_Nspm;
 body.contact_axle_axis_local = tire.axle_axis_local;
 body.friction_anchor = tire.static_friction_anchor;
 }

 ResolvedTireContact resolve(const TireContactSpec& tire,
 const TireSurfaceContact& surface) const override {
 validate(tire);
 validate(surface);
 const double axle_rolling_length = surface.rolling_resistance * tire.radius_m;
 const double base_rolling_length = std::max(tire.rolling_support_moment_arm_m,
 axle_rolling_length);
 const double axle_scale = base_rolling_length > 0.0
 ? axle_rolling_length / base_rolling_length : 1.0;
 const double axle_length = length(tire.axle_axis_local);
 const Vec3 axle{
 tire.axle_axis_local.x / axle_length,
 tire.axle_axis_local.y / axle_length,
 tire.axle_axis_local.z / axle_length,
 };
 return {
 tire.base_friction * std::clamp(surface.friction_scale, 0.0, 2.0),
 base_rolling_length,
 tire.spinning_resistance * tire.radius_m,
 {
 1.0 - (1.0 - axle_scale) * std::abs(axle.x),
 1.0 - (1.0 - axle_scale) * std::abs(axle.y),
 1.0 - (1.0 - axle_scale) * std::abs(axle.z),
 },
 surface.normal_stiffness_Npm,
 surface.normal_damping_Nspm,
 };
 }

 void apply(IPhysicsWorld& world, BodyHandle body, const TireContactSpec& tire,
 const TireSurfaceContact& surface) const override {
 if (world.backend_name() != backend_name()) {
 throw std::invalid_argument("Bullet tire contact requires a Bullet physics world");
 }
 const auto contact = resolve(tire, surface);
 world.set_contact_friction(body, contact.friction,
 contact.rolling_friction_length_m, contact.spinning_friction_length_m);
 world.set_contact_rolling_anisotropy(body, contact.rolling_anisotropy);
 world.set_contact_compliance(body, contact.normal_stiffness_Npm,
 contact.normal_damping_Nspm);
 }
};

} // namespace

std::unique_ptr<ITireContact> make_bullet_tire_contact() {
 return std::make_unique<BulletTireContact>();
}

} // namespace sim::physics
