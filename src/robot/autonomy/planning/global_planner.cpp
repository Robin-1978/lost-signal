#include "robot/autonomy/planning/global_planner.hpp"

#include <cmath>

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

}  // namespace lost_signal::robot::autonomy
