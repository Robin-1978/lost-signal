#pragma once

#include "robot/autonomy/control/follow.hpp"
#include "robot/autonomy/perception/dynamic_obstacle_map.hpp"
#include "robot/runtime/observation.hpp"

#include <cstddef>
#include <memory>
#include <vector>

namespace sim {

class RouteTrackingSystem {
 public:
 RouteTrackingSystem(double kp, double ki, double kd, double output_limit, double integral_limit);

 void reset();
 sim::GuideOut follow_route(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate,
 double lookahead_m, double cruise_mps, double dt_s,
 const sim::GridMap& planning_map, const sim::ChassisCaps& chassis_caps);
 sim::TravelDirection travel_direction(const std::vector<sim::RoutePoint>& path) const;
 // Requests a new global route only after the remaining path has been
 // persistently blocked by static local-map evidence. A blockage explained
 // by a live dynamic track remains the local planner/behavior layer's job.
 bool should_replan(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate, const sim::GridMap& local_map,
 const std::vector<sim::DynamicTrack>& dynamic_tracks, double dt_s);
 double heading_command(double error_rad, double dt_s) { return heading_pid_->update(error_rad, dt_s); }

 std::size_t path_index() const { return guide_state_.idx; }
 void set_path_index(std::size_t index) { guide_state_.idx = index; }
 bool reversing() const { return guide_state_.reversing; }
 void set_reversing(bool reversing) { guide_state_.reversing = reversing; }
 double pid_p() const { return heading_pid_->p(); }
 double pid_i() const { return heading_pid_->i(); }
 double pid_d() const { return heading_pid_->d(); }

 private:
 sim::GuideState guide_state_;
 std::unique_ptr<sim::PID> heading_pid_;
 double static_route_blocked_s_{0.0};
};

} // namespace sim
