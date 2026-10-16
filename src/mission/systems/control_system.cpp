#include "control_system.hpp"

#include <utility>

namespace mission {

ControlSystem::ControlSystem(double kp, double ki, double kd, double output_limit, double integral_limit)
    : heading_pid_(std::make_unique<sim::PID>(kp, ki, kd, output_limit, integral_limit)) {}

void ControlSystem::reset() {
 guide_state_ = {};
 heading_pid_->reset();
}

sim::Vec2 ControlSystem::navigation_target(const std::vector<sim::Vec2>& path,
                                           const sim::EstimatedState& estimate, double lookahead_m,
                                           const sim::GridMap& planning_map, sim::Vec2 fallback_goal) {
 sim::advance_wp(path, guide_state_.idx, estimate.position.x, estimate.position.y, estimate.heading_rad, 0.55,
                 lookahead_m);
 if (guide_state_.idx >= path.size()) {
  return fallback_goal;
 }
 return sim::lookahead_pt(path, guide_state_.idx, estimate.position.x, estimate.position.y, lookahead_m,
                          planning_map);
}

sim::GuideOut ControlSystem::follow_route(const std::vector<sim::Vec2>& path,
                                          const sim::EstimatedState& estimate, double lookahead_m,
                                          double cruise_mps, double dt_s, const sim::GridMap& planning_map,
 const sim::ChassisCaps& chassis_caps) {
 if (chassis_caps.command_space == sim::ChassisCommandSpace::SteeringAngle) {
 return sim::ackermann_guide_step(path, guide_state_, estimate.position.x, estimate.position.y,
 estimate.heading_rad, chassis_caps.geometry.steering_wheel_base_m,
 chassis_caps.geometry.max_steer_rad, lookahead_m,
 cruise_mps, dt_s, planning_map);
 }
 return sim::guide_step(path, guide_state_, estimate.position.x, estimate.position.y, estimate.heading_rad,
                        *heading_pid_, lookahead_m, cruise_mps, dt_s, planning_map, chassis_caps);
}

} // namespace mission
