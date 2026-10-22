#include "sim/providers/sensor_suite.hpp"

namespace lost_signal::sim::providers {

SimSensorSuite::SimSensorSuite(world::World* world)
    : world_(world), motion_(world), lidar_("sim.lidar2d", 72, 10.0f) {}

void SimSensorSuite::register_all(robot::app::IRobot& app) {
  app.register_provider(motion_.descriptor());
  app.register_provider(lidar_.descriptor());
  app.register_provider(imu_.descriptor());
}

bool SimSensorSuite::poll_envelope(robot::iface::ObservationFrame& out) {
  if (world_ == nullptr) return false;
  robot::core::MonotonicNs stamp = static_cast<robot::core::MonotonicNs>(
      world_->truth().step * 20'000'000);
  lidar_.set_step(world_->truth().step, stamp);
  imu_.set_step(world_->truth().step, stamp);

  robot::iface::MotionFeedback fb;
  if (!motion_.poll_feedback(fb)) return false;
  robot::iface::Lidar2DScan scan;
  if (!lidar_.poll(scan)) return false;
  robot::iface::ImuMessage imu;
  if (!imu_.poll(imu)) return false;

  out = robot::iface::make_envelope(fb);
  if (!robot::iface::set_scan(out, scan)) return false;
  if (!robot::iface::set_imu(out, imu)) return false;
  return robot::iface::valid_observation_frame(out);
}

void SimSensorSuite::consume(const robot::iface::MotionCommand& cmd) {
  motion_.consume_command(cmd);
}

}  // namespace lost_signal::sim::providers
