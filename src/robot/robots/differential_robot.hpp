#pragma once

#include <memory>

#include "robot/app/robot.hpp"

namespace lost_signal::robot::robots {

// DifferentialRobot: deployable IRobot for differential-drive bases.
// Thin deployment facade: the inner RobotApplication owns the chassis
// (selected by RobotDescription::chassis_kind), clamps the envelope, and
// translates body commands to actuator setpoints. Owns its inner IRobot via
// unique_ptr because RobotApplication (mutex channel) is non-movable.
class DifferentialRobot : public app::IRobot {
 public:
  explicit DifferentialRobot(std::unique_ptr<app::IRobot> inner);

  core::Status configure() override;
  core::Status activate() override;
  void shutdown() override;
  lifecycle::ModuleState bringup_state() const override;

  void set_goal(
      const ::lost_signal::robot::autonomy::RoutePoint& goal) override {
    inner_->set_goal(goal);
  }
 void set_initial_pose(double x_m, double y_m, double yaw_rad,
 double position_sigma_m, double heading_sigma_rad) override {
 inner_->set_initial_pose(x_m, y_m, yaw_rad, position_sigma_m,
 heading_sigma_rad);
 }
 bool set_operator_arm_effort(
     std::uint8_t count, const std::array<double, 4>& effort) override {
   return inner_->set_operator_arm_effort(count, effort);
 }
 void clear_operator_arm_effort() override {
   inner_->clear_operator_arm_effort();
 }

  iface::GoalHandle request_goal(const iface::TaskRequest& req) override {
    return inner_->request_goal(req);
  }
  iface::GoalStatus goal_status(iface::GoalHandle handle) const override {
    return inner_->goal_status(handle);
  }
  bool cancel_goal(iface::GoalHandle handle) override {
    return inner_->cancel_goal(handle);
  }
  core::Status request_map(const std::string& map_id) override;
  bool register_provider(const iface::EndpointDescriptor& desc) override;
  bool publish_observation(const iface::ObservationFrame& obs) override;
  iface::MotionCommand take_command(core::MonotonicNs now_ns) override;
  iface::ActuatorCommand take_actuator(core::MonotonicNs now_ns) override {
    return inner_->take_actuator(now_ns);
  }
  autonomy::diagnostics::DecisionEvidence last_evidence() const override {
    return inner_->last_evidence();
  }
  iface::MotionCommand tick(core::MonotonicNs now_ns) override;
  iface::GoalStatus current_goal() const override {
    return inner_->current_goal();
  }
  iface::MapSnapshot active_map() const override;
  iface::ChannelHealth channel_health() const override;

 private:
  std::unique_ptr<app::IRobot> inner_;
};

}  // namespace lost_signal::robot::robots
