#include "autonomy/mapping/terrain.hpp"

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

} // namespace

TerrainSample TerrainModel::sample(Vec2 position) const {
 for (auto patch = patches_.rbegin(); patch != patches_.rend(); ++patch) {
  if (!contains(*patch, position)) {
   continue;
  }
  return {
    patch->base_height_m + patch->grade_x * (position.x - patch->min.x) +
      patch->grade_y * (position.y - patch->min.y),
    patch->grade_x,
    patch->grade_y,
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

} // namespace sim
