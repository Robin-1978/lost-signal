#include "sim/robot/sim_robot.hpp"

#include <algorithm>

namespace lost_signal::sim::robot {

namespace r = ::lost_signal::robot;

SimRobot::SimRobot()
    : suite_(&world_),
      app_(std::make_unique<providers::StaticMapProvider>(
          &world_, -10.0, -10.0, 20.0, 20.0, 0.25)) {
  suite_.register_all(app_);
  // Harness bringup: ordered configure -> activate; a failed module blocks.
  (void)app_.configure();
  (void)app_.activate();
}

r::core::Status SimRobot::configure() { return app_.configure(); }

r::core::Status SimRobot::activate() { return app_.activate(); }

void SimRobot::shutdown() { app_.shutdown(); }

r::lifecycle::ModuleState SimRobot::bringup_state() const {
  return app_.bringup_state();
}

r::core::Status SimRobot::request_map(const std::string& map_id) {
  return app_.request_map(map_id);
}

bool SimRobot::register_provider(const r::iface::EndpointDescriptor& desc) {
  return app_.register_provider(desc);
}

bool SimRobot::publish_observation(const r::iface::ObservationFrame& obs) {
  return app_.publish_observation(obs);
}

r::iface::MotionCommand SimRobot::take_command(r::core::MonotonicNs now_ns) {
  return app_.take_command(now_ns);
}

r::iface::MotionCommand SimRobot::tick(r::core::MonotonicNs now_ns) {
  // Decision time is the caller's simulation time; the observation keeps
 // its acquisition stamp so delivery latency remains visible.
 world_.step(0.02);
 const r::core::MonotonicNs plant_ns =
 world_.sim_time_ns();
 // The world just stepped, so the decision clock is the plant clock. A
 // caller clock ahead of the plant is honored; a lagging one is not
 // allowed to make a fresh scan look like it came from the future.
 const r::core::MonotonicNs decision_ns =
 (now_ns != r::core::kInvalidTime && now_ns > plant_ns) ? now_ns
 : plant_ns;
  r::iface::ObservationFrame obs;
  if (!suite_.poll_envelope(obs)) {
 return r::iface::MotionCommand::SafeStop(world_.sim_time_ns());
  }
  if (!app_.publish_observation(obs)) {
    return r::iface::MotionCommand::SafeStop(decision_ns);
  }
  r::iface::MotionCommand cmd = app_.tick(decision_ns);
  r::iface::ActuatorCommand act = app_.take_actuator(decision_ns);
  suite_.consume(act);
  return cmd;
}

r::iface::MapSnapshot SimRobot::active_map() const {
  return app_.active_map();
}

r::iface::ChannelHealth SimRobot::channel_health() const {
  return app_.channel_health();
}

}  // namespace lost_signal::sim::robot
