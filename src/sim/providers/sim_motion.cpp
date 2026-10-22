#include "sim/providers/sim_motion.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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
  // Encoder view: quantize angles to counts, differentiate over the stamp
  // delta. The count quantum comes from the motor params (same DriveProfile
  // contract the ROBOT side reads); non-positive counts mean no encoder.
  const int counts = world_->motor_params().encoder_counts_per_revolution;
  const double quantum_rad =
      counts > 0 ? 2.0 * M_PI / static_cast<double>(counts) : 0.0;
  const double dt_s =
      have_prev_ ? static_cast<double>(out.stamp_ns - prev_stamp_ns_) * 1e-9
                  : 0.0;
  for (int i = 0; i < 4; ++i) {
    const double drive_q =
        quantum_rad > 0.0
            ? std::round(world_->truth().drive_angle_rad[i] / quantum_rad) *
                  quantum_rad
            : world_->truth().drive_angle_rad[i];
    const double steer_q =
        quantum_rad > 0.0
            ? std::round(world_->truth().steer_rad[i] / quantum_rad) *
                  quantum_rad
            : world_->truth().steer_rad[i];
    if (have_prev_ && dt_s > 0.0) {
      prev_drive_vel_radps_[i] =
          (drive_q - prev_drive_angle_rad_[i]) / dt_s;
      prev_steer_vel_radps_[i] =
          (steer_q - prev_steer_angle_rad_[i]) / dt_s;
    } else if (!have_prev_) {
      prev_drive_vel_radps_[i] = 0.0;  // just powered: no history yet
      prev_steer_vel_radps_[i] = 0.0;
    }
    // Repeated stamp: hold the previous velocity (never divide by zero).
    prev_drive_angle_rad_[i] = drive_q;
    prev_steer_angle_rad_[i] = steer_q;
    out.drive_radps[i] = 0.0;
    out.steer_rad[i] = 0.0;
    out.steer_vel_radps[i] = 0.0;
  }
  for (int i = 0; i < out.drive_count; ++i) {
    out.drive_radps[i] = prev_drive_vel_radps_[i];
  }
  for (int i = 0; i < out.steer_count; ++i) {
    out.steer_rad[i] = prev_steer_angle_rad_[i];
    out.steer_vel_radps[i] = prev_steer_vel_radps_[i];
  }
  // Traction flags ride the same poll (reported, never clamped). Stale
  // channels stay false: only the mode-implied drives carry a flag.
  out.drive_slipping = {};
  for (int i = 0; i < out.drive_count && i < 4; ++i) {
    out.drive_slipping[i] = world_->truth().drive_slipping[i];
  }
  have_prev_ = true;
  prev_stamp_ns_ = out.stamp_ns;
  return out.valid();
}

}  // namespace lost_signal::sim::providers
