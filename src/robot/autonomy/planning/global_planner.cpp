#include "robot/autonomy/planning/global_planner.hpp"

#include <cmath>

#include "robot/autonomy/planning/ackermann_planner.hpp"
#include "robot/autonomy/planning/astar_planner.hpp"

namespace lost_signal::robot::autonomy {

Route StraightLinePlanner::plan(const EstimatedState& start,
                                const RoutePoint& goal,
                                const iface::MapSnapshot& /*map*/) {
  Route r;
  if (!start.valid) return r;
  double dx = goal.x_m - start.x_m;
  double dy = goal.y_m - start.y_m;
  double dist = std::hypot(dx, dy);
  double yaw = std::atan2(dy, dx);
  std::size_t n = static_cast<std::size_t>(dist / 0.5) + 1;
  if (n < 2) n = 2;
  if (n > 1024) n = 1024;
  r.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    double s = static_cast<double>(i) / (n - 1);
    r.points.push_back({start.x_m + dx * s, start.y_m + dy * s, yaw});
  }
 r.valid = true;
 return r;
}

std::unique_ptr<IGlobalPlanner> make_global_planner(
 const platform::ChassisLimits& limits, double wheel_base_m,
 const platform::NavigationSafetyPolicy& navigation) {
 if (platform::chassis_is_ackermann(limits)) {
 AckermannPlanner::Config config;
 config.wheel_base_m = std::isfinite(wheel_base_m) && wheel_base_m > 0.0
 ? wheel_base_m
 : config.wheel_base_m;
 config.max_steer_rad = std::isfinite(limits.max_steer_rad)
 ? limits.max_steer_rad
 : config.max_steer_rad;
 config.hard_clearance_margin_m = navigation.clearance_margin_m;
 config.soft_inflation_radius_m = navigation.soft_inflation_radius_m;
 if (std::isfinite(limits.footprint_radius_m) &&
 limits.footprint_radius_m > 0.0) {
 config.footprint_radius_m = limits.footprint_radius_m;
 }
 return std::make_unique<AckermannPlanner>(config);
 }
 AStarPlanner::Config config;
 config.hard_clearance_margin_m = navigation.clearance_margin_m;
 config.soft_inflation_radius_m = navigation.soft_inflation_radius_m;
 if (std::isfinite(limits.footprint_radius_m) &&
 limits.footprint_radius_m > 0.0) {
 config.footprint_radius_m = limits.footprint_radius_m;
 }
 return std::make_unique<AStarPlanner>(config);
}

}  // namespace lost_signal::robot::autonomy
