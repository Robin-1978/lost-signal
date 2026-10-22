#pragma once

#include "robot/app/robot.hpp"
#include "robot/app/robot_application.hpp"
#include "sim/providers/sensor_suite.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::robot {

// SimRobot: IRobot harness backed by World + sim providers. Same facade as
// the deployable robot, so tools/tests program against IRobot* without
// branching on deployment. Truth stays inside; only LSRI messages cross.
class SimRobot : public ::lost_signal::robot::app::IRobot {
 public:
  SimRobot();

  ::lost_signal::robot::core::Status configure() override;
  ::lost_signal::robot::core::Status activate() override;
  void shutdown() override;
  ::lost_signal::robot::lifecycle::ModuleState bringup_state() const override;

  void set_goal(
      const ::lost_signal::robot::autonomy::RoutePoint& goal) override {
    app_.set_goal(goal);
  }

  ::lost_signal::robot::core::Status submit_task(
      const ::lost_signal::robot::iface::TaskRequest& req) override;
  ::lost_signal::robot::core::Status request_map(
      const std::string& map_id) override;
  bool register_provider(
      const ::lost_signal::robot::iface::EndpointDescriptor& desc) override;
  bool publish_observation(
      const ::lost_signal::robot::iface::ObservationFrame& obs) override;
  ::lost_signal::robot::iface::MotionCommand take_command(
      ::lost_signal::robot::core::MonotonicNs now_ns) override;
  // One self-stepping cycle: world -> suite envelope -> app.tick -> plant.
  ::lost_signal::robot::iface::MotionCommand tick(
      ::lost_signal::robot::core::MonotonicNs now_ns) override;
  ::lost_signal::robot::iface::TaskStatus task_status() const override;
  ::lost_signal::robot::iface::MapSnapshot active_map() const override;
  ::lost_signal::robot::iface::ChannelHealth channel_health() const override;

  const world::World& world() const { return world_; }

 private:
  world::World world_;
  providers::SimSensorSuite suite_;
  ::lost_signal::robot::app::RobotApplication app_;
};

}  // namespace lost_signal::sim::robot
