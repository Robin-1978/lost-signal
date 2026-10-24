#include "robot/autonomy/planning/route_smoothing.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {
namespace {

bool finite_point(const RoutePoint& p) {
 return std::isfinite(p.x_m) && std::isfinite(p.y_m) &&
 std::isfinite(p.yaw_rad) && (p.gear == -1 || p.gear == 0 || p.gear == 1);
}

double path_heading(const RoutePoint& a, const RoutePoint& b, int gear) {
 const double tangent = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
 return core::wrap_angle(tangent + (gear < 0 ? core::kPi : 0.0));
}

} // namespace

bool smooth_ackermann_route(Route& route, const RouteSmoothingConfig& config) {
 if (!route.valid || route.points.size() < 2 ||
 !(config.wheel_base_m > 0.0) || !std::isfinite(config.wheel_base_m) ||
 !(config.max_steer_rad > 0.0) ||
 !std::isfinite(config.max_steer_rad) ||
 !(config.max_heading_step_rad > 0.0) ||
 !std::isfinite(config.max_heading_step_rad) ||
 !(config.max_curvature_delta_per_m > 0.0) ||
 !std::isfinite(config.max_curvature_delta_per_m)) {
 return false;
 }
 for (const auto& p : route.points) {
 if (!finite_point(p)) return false;
 }
 const double max_curvature = std::tan(config.max_steer_rad) /
 config.wheel_base_m;
 for (std::size_t i = 1; i + 1 < route.points.size(); ++i) {
 auto& p = route.points[i];
 const auto& prev = route.points[i - 1];
 const auto& next = route.points[i + 1];
 if (p.gear != 0 && prev.gear != 0 && p.gear != prev.gear) continue;
 const int gear = p.gear != 0 ? p.gear :
 (prev.gear != 0 ? prev.gear : next.gear);
 const double incoming = path_heading(prev, p, gear);
 const double outgoing = path_heading(p, next, gear);
 // A circular mean avoids a +/-pi discontinuity at the wrap boundary.
 const double blended = std::atan2(std::sin(incoming) + std::sin(outgoing),
 std::cos(incoming) + std::cos(outgoing));
 p.yaw_rad = core::wrap_angle(blended);
 }
 for (std::size_t i = 0; i + 1 < route.points.size(); ++i) {
 const auto& a = route.points[i];
 const auto& b = route.points[i + 1];
 const double dist = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
 if (!(dist > 1e-6)) return false;
 if (a.gear != 0 && b.gear != 0 && a.gear != b.gear) continue;
 const double heading_step = std::abs(core::wrap_angle(b.yaw_rad - a.yaw_rad));
 const double curvature = heading_step / dist;
 if (heading_step > config.max_heading_step_rad + 1e-9 ||
 curvature > max_curvature + 1e-6) return false;
 if (i > 0 && route.points[i - 1].gear == a.gear && a.gear == b.gear) {
 const auto& before = route.points[i - 1];
 const double before_dist = std::hypot(a.x_m - before.x_m,
 a.y_m - before.y_m);
 if (before_dist > 1e-6) {
 const double before_curvature =
 std::abs(core::wrap_angle(a.yaw_rad - before.yaw_rad)) / before_dist;
 if (std::abs(curvature - before_curvature) /
 dist > config.max_curvature_delta_per_m + 1e-6) return false;
 }
 }
 }
 return true;
}

} // namespace lost_signal::robot::autonomy
