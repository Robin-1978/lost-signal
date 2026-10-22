#include "sim/providers/sim_platform.hpp"

namespace lost_signal::sim::providers {

SimPlatformProvider::SimPlatformProvider(world::World* world) : world_(world) {}

const char* SimPlatformProvider::endpoint_id() const { return "sim.platform"; }

bool SimPlatformProvider::poll_observation(robot::iface::ObservationFrame& out) {
  if (world_ == nullptr) return false;
  seq_ += 1;
  out.sequence = seq_;
  out.stamp_ns = static_cast<robot::core::MonotonicNs>(world_->truth().step * 20'000'000);
  out.source = endpoint_id();
  out.frame = robot::core::Frame::kLidarLink;
  // Synthetic measurement derived from truth, never truth itself.
  out.scan_valid = true;
  out.ranges_m = {5.0f, 5.0f, 5.0f};
  out.odometry_valid = true;
  out.forward_mps = world_->truth().forward_mps;
  out.yaw_rate_radps = 0.0;
  out.imu_valid = true;
  return robot::iface::valid_observation_frame(out);
}

void SimPlatformProvider::consume_command(const robot::iface::MotionCommand& cmd) {
  if (world_ == nullptr) return;
  // v1 skeleton: sim applies only validated commands; truth stays in sim.
  (void)cmd;
}

}  // namespace lost_signal::sim::providers
