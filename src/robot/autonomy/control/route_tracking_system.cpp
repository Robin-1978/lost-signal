#include "robot/autonomy/control/route_tracking_system.hpp"

#include <utility>

namespace sim {

RouteTrackingSystem::RouteTrackingSystem(double kp, double ki, double kd, double output_limit, double integral_limit)
    : heading_pid_(std::make_unique<sim::PID>(kp, ki, kd, output_limit, integral_limit)) {}

void RouteTrackingSystem::reset() {
 guide_state_ = {};
 heading_pid_->reset();
}

sim::Vec2 RouteTrackingSystem::navigation_target(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate,
 double lookahead_m,
                                           const sim::GridMap& planning_map, sim::Vec2 fallback_goal) {
 const bool gear_changes = sim::path_has_reverse(path);
 std::size_t section_end = path.size();
 if (gear_changes) {
 section_end = sim::path_direction_section_end(path, guide_state_.idx);
 sim::advance_wp_until(path, guide_state_.idx, section_end, estimate.position.x,
 estimate.position.y, estimate.heading_rad, 0.55);
 } else {
 sim::advance_wp(path, guide_state_.idx, estimate.position.x, estimate.position.y,
 estimate.heading_rad, 0.55, lookahead_m);
 }
 if (guide_state_.idx >= path.size()) {
  return fallback_goal;
 }
 if (gear_changes) {
 section_end = sim::path_direction_section_end(path, guide_state_.idx);
 return sim::lookahead_pt_until(path, guide_state_.idx, section_end, estimate.position.x,
 estimate.position.y, lookahead_m, planning_map);
 }
 return sim::lookahead_pt(path, guide_state_.idx, estimate.position.x, estimate.position.y,
 lookahead_m, planning_map);
}

sim::GuideOut RouteTrackingSystem::follow_route(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate,
 double lookahead_m,
                                          double cruise_mps, double dt_s, const sim::GridMap& planning_map,
 const sim::ChassisCaps& chassis_caps) {
 if (chassis_caps.command_space == sim::ChassisCommandSpace::SteeringAngle) {
 return sim::ackermann_guide_step(path, guide_state_, estimate.position.x,
 estimate.position.y, estimate.heading_rad, chassis_caps.geometry.steering_wheel_base_m,
 chassis_caps.geometry.max_steer_rad, lookahead_m,
 cruise_mps, dt_s, planning_map);
 }
 return sim::guide_step(path, guide_state_, estimate.position.x, estimate.position.y, estimate.heading_rad,
                        *heading_pid_, lookahead_m, cruise_mps, dt_s, planning_map, chassis_caps);
}

sim::TravelDirection RouteTrackingSystem::travel_direction(const std::vector<sim::RoutePoint>& path) const {
 return sim::path_travel_direction(path, guide_state_.idx);
}

} // namespace sim
