#include "robot/autonomy/mapping/terrain.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {
namespace {

constexpr double kGravityMps2 = 9.81;
// Height patches meet at their authored edges. A change steeper than this is
// a retaining wall or platform drop, not a driveable ramp. Sampling the
// segment endpoints keeps A* from "climbing" onto an elevated deck through
// its side just because the deck itself is flat.
constexpr double kMaximumContinuousGrade = 0.85;

bool contains(const HeightPatch& patch, Vec2 position) {
 return position.x >= patch.min.x && position.x < patch.max.x && position.y >= patch.min.y &&
        position.y < patch.max.y;
}

bool sample_support(const SupportSurface& support, Vec2 position, TerrainSample& result) {
 const double dx = position.x - support.center.x;
 const double dy = position.y - support.center.y;
 const double cosine = std::cos(support.yaw_rad);
 const double sine = std::sin(support.yaw_rad);
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 if (std::abs(local_x) >= support.half_length_m ||
 std::abs(local_y) >= support.half_width_m) {
 return false;
 }
 result = {
 support.center_height_m + support.grade_along_local_x * local_x,
 support.grade_along_local_x * cosine,
 support.grade_along_local_x * sine,
 0.0,
 };
 return true;
}


} // namespace

TerrainSample TerrainModel::sample(Vec2 position) const {
 // Runtime supports are later world revisions and therefore overlay authored
 // terrain wherever their bounded footprint exists.
 for (auto support = supports_.rbegin(); support != supports_.rend(); ++support) {
 TerrainSample result;
 if (sample_support(*support, position, result))
 return result;
 }
 for (auto patch = patches_.rbegin(); patch != patches_.rend(); ++patch) {
  if (!contains(*patch, position)) {
   continue;
  }
 const double dx = position.x - patch->min.x;
 const double dy = position.y - patch->min.y;
 const ReliefSample relief = rough_relief(patch->roughness,
 patch->max.x - patch->min.x, patch->max.y - patch->min.y, dx, dy);
 return {
 patch->base_height_m + patch->grade_x * dx + patch->grade_y * dy +
 patch->grade_xy_per_m * dx * dy + relief.height_m,
 patch->grade_x + patch->grade_xy_per_m * dy + relief.gradient_x,
 patch->grade_y + patch->grade_xy_per_m * dx + relief.gradient_y,
    patch->roughness,
  };
 }
 return {};
}

double TerrainModel::traversal_cost(Vec2 from, Vec2 to, const TerrainCapability& capability) const {
 const double dx = to.x - from.x;
 const double dy = to.y - from.y;
 const double distance = std::hypot(dx, dy);
 if (distance < 1e-12) {
  return 1.0;
 }
 const auto from_terrain = sample(from);
 const auto to_terrain = sample(to);
 const double segment_grade = (to_terrain.height_m - from_terrain.height_m) / distance;
 if (std::abs(segment_grade) > kMaximumContinuousGrade) {
 return std::numeric_limits<double>::infinity();
 }
 const double heading = std::atan2(dy, dx);
 const TerrainSample terrain = sample({(from.x + to.x) * 0.5, (from.y + to.y) * 0.5});
 const double forward_grade = std::max(longitudinal_grade(terrain, heading), segment_grade);
 const double cross_grade = lateral_grade(terrain, heading);
 const double slope_rad = std::atan(forward_grade);
 const double cross_rad = std::atan(std::abs(cross_grade));
 if (cross_rad > capability.max_cross_slope_rad) {
  return std::numeric_limits<double>::infinity();
 }
 const double required_force = capability.mass_kg * kGravityMps2 * std::max(0.0, std::sin(slope_rad));
 const double traction_force =
   capability.mass_kg * kGravityMps2 * std::max(0.0, capability.friction_mu) * std::cos(slope_rad);
 const double usable_force = std::min(capability.available_drive_force_N, traction_force);
 if (required_force > usable_force * 1.25 + 1e-9) {
  return std::numeric_limits<double>::infinity();
 }
 return 1.0 + capability.uphill_cost_weight * std::max(0.0, forward_grade) +
        capability.cross_slope_cost_weight * std::abs(cross_grade) +
        capability.roughness_cost_weight * std::max(0.0, terrain.roughness);
}

double longitudinal_grade(const TerrainSample& terrain, double heading_rad) {
 return terrain.grade_x * std::cos(heading_rad) + terrain.grade_y * std::sin(heading_rad);
}

double lateral_grade(const TerrainSample& terrain, double heading_rad) {
 return -terrain.grade_x * std::sin(heading_rad) + terrain.grade_y * std::cos(heading_rad);
}

double static_rollover_angle(double support_width_m, double center_of_mass_height_m) {
 return std::atan2(std::max(0.01, support_width_m) * 0.5, std::max(0.01, center_of_mass_height_m));
}

double rollover_risk(double cross_grade, double support_width_m, double center_of_mass_height_m) {
 return std::abs(std::atan(cross_grade)) /
        std::max(0.05, static_rollover_angle(support_width_m, center_of_mass_height_m));
}

double roughness_lateral_acceleration(double roughness_m, double speed_mps, double support_width_m) {
 // Peak acceleration of a smooth bump whose amplitude is roughness_m and
 // whose half-wave spans roughly the vehicle support width. This is a
 // deterministic stability envelope rather than random pose jitter.
 const double width = std::max(0.1, support_width_m);
 return 4.0 * std::max(0.0, roughness_m) * speed_mps * speed_mps / width;
}

double dynamic_rollover_risk(double cross_grade, double lateral_accel_mps2, double support_width_m,
                             double center_of_mass_height_m) {
 const double surface_angle = std::atan(cross_grade);
 const double lateral_load = kGravityMps2 * std::sin(surface_angle) + lateral_accel_mps2;
 const double normal_load = kGravityMps2 * std::cos(surface_angle);
 const double effective_angle = std::atan2(std::abs(lateral_load), std::max(0.05, normal_load));
 return effective_angle / std::max(0.05, static_rollover_angle(support_width_m, center_of_mass_height_m));
}

double roughness_vertical_acceleration(double roughness_m, double speed_mps, double support_base_m) {
 const double base = std::max(0.1, support_base_m);
 const double wavenumber = 2.0 * kPi / base;
 return wavenumber * wavenumber * std::max(0.0, roughness_m) * speed_mps * speed_mps;
}

double roughness_ride_speed_limit(double roughness_m, double support_base_m, double gravity_fraction) {
 const double per_speed_squared =
 roughness_vertical_acceleration(roughness_m, 1.0, support_base_m);
 if (!(per_speed_squared > 1e-9)) {
 return std::numeric_limits<double>::infinity();
 }
 return std::sqrt(std::max(0.0, gravity_fraction) * kGravityMps2 / per_speed_squared);
}

double support_polygon_risk(Vec2 load_projection, Vec2 unloaded_projection,
 const std::vector<Vec2>& support_polygon) {
 if (support_polygon.size() < 3)
 return std::numeric_limits<double>::infinity();
 double twice_area = 0.0;
 for (std::size_t index = 0; index < support_polygon.size(); ++index) {
 const auto& a = support_polygon[index];
 const auto& b = support_polygon[(index + 1) % support_polygon.size()];
 twice_area += a.x * b.y - a.y * b.x;
 }
 if (std::abs(twice_area) < 1e-12)
 return std::numeric_limits<double>::infinity();
 const double orientation = twice_area > 0.0 ? 1.0 : -1.0;
 double risk = 0.0;
 for (std::size_t index = 0; index < support_polygon.size(); ++index) {
 const auto& a = support_polygon[index];
 const auto& b = support_polygon[(index + 1) % support_polygon.size()];
 const Vec2 edge{b.x - a.x, b.y - a.y};
 const auto signed_inside = [&](Vec2 point) {
 return orientation * (edge.x * (point.y - a.y) - edge.y * (point.x - a.x));
 };
 const double reference_distance = signed_inside(unloaded_projection);
 if (!(reference_distance > 1e-12))
 return std::numeric_limits<double>::infinity();
 const double loaded_distance = signed_inside(load_projection);
 risk = std::max(risk, 1.0 - loaded_distance / reference_distance);
 }
 return std::max(0.0, risk);
}

double longitudinal_stability_acceleration_limit(double longitudinal_grade,
 double front_support_m, double rear_support_m, double center_of_mass_height_m,
 double margin_fraction) {
 const double height = std::max(0.01, center_of_mass_height_m);
 const double front = std::max(0.0, front_support_m);
 const double rear = std::max(0.0, rear_support_m);
 const double slope = std::atan(longitudinal_grade);
 const double gravity_normal = kGravityMps2 * std::cos(slope);
 const double gravity_forward = kGravityMps2 * std::sin(slope);
 // Positive acceleration shifts normal load rearward; negative acceleration
 // shifts it forward. Grade contributes before any commanded acceleration.
 const double margin = std::clamp(margin_fraction, 0.0, 1.0);
 // The margin applies to total load-line travel from the unloaded COM, not
 // only to the actuator acceleration left after gravity. On a downhill grade
 // gravity may already consume most of the front-support allowance; scaling
 // the residual limit would silently give that gravity term back.
 const double forward_limit =
 margin * gravity_normal * rear / height - gravity_forward;
 const double braking_limit =
 margin * gravity_normal * front / height + gravity_forward;
 return std::max(0.0, std::min(forward_limit, braking_limit));
}

} // namespace sim
