#pragma once

#include <memory>

#include "robot/app/robot.hpp"
#include "robot/platform/chassis.hpp"

namespace lost_signal::robot::robots {

// DifferentialRobot: deployable IRobot for differential-drive bases.
// Same facade as RobotApplication, plus chassis-envelope clamping on every
// tick so a planner can never publish a yaw rate the base cannot track.
// Owns its inner IRobot via unique_ptr because RobotApplication (mutex
// channel) is non-movable.
class DifferentialRobot : public app::IRobot {
 public:
  DifferentialRobot(std::unique_ptr<app::IRobot> inner,
                    platform::DifferentialChassis chassis);

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
  iface::MotionCommand tick(core::MonotonicNs now_ns) override;
  iface::TaskStatus task_status() const override;
  iface::MapSnapshot active_map() const override;
  iface::ChannelHealth channel_health() const override;

  const platform::DifferentialChassis& chassis() const { return chassis_; }

 private:
  std::unique_ptr<app::IRobot> inner_;
  platform::DifferentialChassis chassis_;
};

}  // namespace lost_signal::robot::robots
