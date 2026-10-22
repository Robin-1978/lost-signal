#include "sim/providers/sim_motion.hpp"

namespace lost_signal::sim::providers {

SimMotionEndpoint::SimMotionEndpoint(world::World* world) : world_(world) {
  desc_.endpoint_id = "sim.motion";
  desc_.role = robot::iface::EndpointRole::kEncoder;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "base_link";
}

const robot::iface::EndpointDescriptor& SimMotionEndpoint::descriptor()
    const {
  return desc_;
}

void SimMotionEndpoint::consume_command(
    const robot::iface::ActuatorCommand& cmd) {
  if (world_ == nullptr) return;
  if (!robot::iface::valid_actuator_command(cmd, robot::core::kInvalidTime)) {
    return;  // malformed duties never reach the plant
  }
  world_->apply(cmd);
}

bool SimMotionEndpoint::poll_feedback(
    robot::iface::MotionFeedback& out) {
  if (world_ == nullptr) return false;
  seq_ += 1;
  out.sequence = seq_;
  out.stamp_ns = static_cast<robot::core::MonotonicNs>(world_->truth().step *
                                                       20'000'000);
  out.source = desc_.endpoint_id;
  out.frame = robot::core::Frame::kBaseLink;
  out.odometry_valid = true;
  out.forward_mps = world_->truth().forward_mps;
  out.yaw_rate_radps = world_->truth().yaw_rate_radps;
  out.drive_count = world_->truth().drive_count;
  out.steer_count = world_->truth().steer_count;
  out.drive_radps = world_->truth().drive_radps;
  out.steer_rad = world_->truth().steer_rad;
  out.steer_vel_radps = world_->truth().steer_vel_radps;
  return out.valid();
}

}  // namespace lost_signal::sim::providers
