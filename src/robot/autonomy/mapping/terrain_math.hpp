#pragma once

// ROBOT-owned terrain math (pure functions over the surveyed elevation
// prior + hardware numbers, never SIM truth). The survey carries elevation
// only (no friction, contact, or wetness); roughness here is the ROBOT's
// own belief from its IMU observer (traversal.hpp), never the author's
// amplitude. Formulas mirror main's mapping/terrain (gravity is physics):
// grade splits, rollover envelope, bump-excitation speed limits, and the
// ground-return classifier shared by the validator and the mapper so the
// two cannot disagree on what counts as ground.

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/core/geometry.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

namespace {

// Standard gravity: physics, not a tuned constant.
constexpr double kTerrainGravityMps2 = 9.81;

}  // namespace

// One shared sensor-plane tolerance for all surveyed-ground classifiers.  A
// return is only cleared as terrain when the public elevation prior agrees
// with the measured sensor plane within this declared resolution budget.
inline constexpr double kTerrainGroundToleranceM = 0.15;

// Along-track grade (rise/run) of the surveyed surface under a heading.
// Finite differences at the survey resolution; unknown elevation reads 0
// (flat assumption — gates that must fail closed check has_elevation).
inline double survey_longitudinal_grade(const iface::MapSnapshot& map,
                                        double x_m, double y_m,
                                        double yaw_rad) {
  if (!map.has_elevation()) return 0.0;
  const double eps = std::max(0.1, map.resolution_m);
  const double c = std::cos(yaw_rad), s = std::sin(yaw_rad);
  const float ahead = map.elevation_at(x_m + 0.5 * eps * c, y_m + 0.5 * eps * s);
  const float behind =
      map.elevation_at(x_m - 0.5 * eps * c, y_m - 0.5 * eps * s);
  if (!std::isfinite(ahead) || !std::isfinite(behind)) return 0.0;
  return static_cast<double>(ahead - behind) / eps;
}

// Cross-track grade (positive = uphill to the left).
inline double survey_lateral_grade(const iface::MapSnapshot& map, double x_m,
                                   double y_m, double yaw_rad) {
  return survey_longitudinal_grade(map, x_m, y_m, yaw_rad + core::kPi / 2.0);
}

// Surveyed grade along the estimated heading (navigation prior, never
// live physics). NaN = unsurveyed = flat assumption, erring toward caution
// on grades the survey never measured. Pure transform for the coordinator
// to wire (it owns no adaptation policy); the governor owns the evidence.
inline double survey_pitch_along(const iface::MapSnapshot& map,
                                 double x_m, double y_m, double yaw_rad) {
  if (!map.has_elevation()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  const float ahead = map.elevation_at(x_m + 0.5 * c, y_m + 0.5 * s);
  const float behind = map.elevation_at(x_m - 0.5 * c, y_m - 0.5 * s);
  if (!std::isfinite(ahead) || !std::isfinite(behind)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::atan2(static_cast<double>(ahead - behind), 1.0);
}

// Ground-return classifier: a scan endpoint is surveyed ground (not an
// obstacle) when the survey elevation under it agrees with the sensor
// height within tolerance. A horizontal beam meets an up-ramp where the
// surface crosses the sensor plane (|diff| ~ 0); a wall face endpoint sits
// under a surveyed prism top (diff > 0) or past it. Unknown survey (NaN)
// is never ground — fail closed, today's behavior.
inline bool endpoint_is_survey_ground(const iface::MapSnapshot& map,
                                      double end_x_m, double end_y_m,
                                      double sensor_z_m, double tol_m) {
  if (!map.has_elevation() || !(tol_m > 0.0) ||
      !std::isfinite(sensor_z_m)) {
    return false;
  }
  const float e = map.elevation_at(end_x_m, end_y_m);
  if (!std::isfinite(e)) return false;
  return std::abs(static_cast<double>(e) - sensor_z_m) <= tol_m;
}

// Static tip-over angle from the support width and COM height.
inline double static_rollover_angle(double support_width_m,
                                    double com_height_m) {
  return std::atan2(std::max(0.01, support_width_m) * 0.5,
                    std::max(0.01, com_height_m));
}

// Rollover risk in [0, ...]: 1.0 at the tip-over angle. Cross-grade only.
inline double rollover_risk(double cross_grade, double support_width_m,
                            double com_height_m) {
  return std::abs(std::atan(cross_grade)) /
         std::max(0.05,
                  static_rollover_angle(support_width_m, com_height_m));
}

// Vertical bump excitation of roughness amplitude at speed over a support
// base (deterministic stability envelope, main's form).
inline double roughness_vertical_acceleration(double roughness_m,
                                              double speed_mps,
                                              double support_base_m) {
  const double base = std::max(0.1, support_base_m);
  const double wavenumber = 2.0 * core::kPi / base;
  return wavenumber * wavenumber * std::max(0.0, roughness_m) * speed_mps *
         speed_mps;
}

// Speed at which bump excitation reaches gravity_fraction * g. Infinity on
// smooth ground (no limit from roughness).
inline double roughness_ride_speed_limit(double roughness_m,
                                         double support_base_m,
                                         double gravity_fraction) {
  const double per_speed_squared =
      roughness_vertical_acceleration(roughness_m, 1.0, support_base_m);
  if (!(per_speed_squared > 1e-9)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(std::max(0.0, gravity_fraction) * kTerrainGravityMps2 /
                   per_speed_squared);
}

// Dynamic rollover risk with lateral acceleration added to the surface
// load (main's form): surface tilt plus cornering/rocking accel over the
// same support envelope.
inline double dynamic_rollover_risk(double cross_grade,
                                    double lateral_accel_mps2,
                                    double support_width_m,
                                    double com_height_m) {
  const double surface_angle = std::atan(cross_grade);
  const double lateral_load =
      kTerrainGravityMps2 * std::sin(surface_angle) + lateral_accel_mps2;
  const double normal_load =
      kTerrainGravityMps2 * std::cos(surface_angle);
  const double effective_angle = std::atan2(
      std::abs(lateral_load), std::max(0.05, normal_load));
  return effective_angle /
         std::max(0.05,
                  static_rollover_angle(support_width_m, com_height_m));
}

}  // namespace lost_signal::robot::autonomy
