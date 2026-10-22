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

  core::Status submit_task(const iface::TaskRequest& req) override;
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
  iface::TaskStatus task_status() const override;
  iface::MapSnapshot active_map() const override;
  iface::ChannelHealth channel_health() const override;

 private:
  std::unique_ptr<app::IRobot> inner_;
};

}  // namespace lost_signal::robot::robots
