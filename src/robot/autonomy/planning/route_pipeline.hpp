#pragma once

#include <limits>
#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// Event-driven route refresh policy. The coordinator owns lifecycle and
// transport ordering; this module owns the decision of when a global planner
// is called and replaces the active route atomically.
class RoutePlanningPipeline {
 public:
 void reset() {
 pending_ = false;
 failed_ = false;
 has_request_ = false;
 diagnostic_.clear();
 }
 bool pending() const { return pending_; }
 bool failed() const { return failed_; }
 const std::string& diagnostic() const { return diagnostic_; }
 bool advance(IGlobalPlanner& planner, const EstimatedState& start,
 const RoutePoint& goal, bool has_goal,
 const iface::MapSnapshot& map, bool force_replan,
 Route& active_route,
 double maximum_route_deviation_m =
 std::numeric_limits<double>::infinity());
  static bool refresh(IGlobalPlanner& planner, const EstimatedState& start,
                      const RoutePoint& goal, bool has_goal,
                      const iface::MapSnapshot& map, bool force_replan,
                      Route& active_route);

 private:
 bool same_request(const RoutePoint& goal,
 const iface::MapSnapshot& map) const;
 bool failed_start_changed(const EstimatedState& start,
 const iface::MapSnapshot& map) const;
 bool pending_ = false;
 bool failed_ = false;
 bool has_request_ = false;
 RoutePoint requested_goal_;
 std::string requested_map_id_;
 std::uint64_t requested_map_revision_ = 0;
 double failed_start_x_m_ = 0.0;
 double failed_start_y_m_ = 0.0;
 bool failed_start_valid_ = false;
 std::string diagnostic_;
};

}  // namespace lost_signal::robot::autonomy
