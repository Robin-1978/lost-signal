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
 out.stamp_ns = world_->sim_time_ns();
  out.source = desc_.endpoint_id;
  out.frame = robot::core::Frame::kBaseLink;
  out.odometry_valid = true;
  out.drive_count = world_->truth().drive_count;
  out.steer_count = world_->truth().steer_count;
 out.arm_count = static_cast<std::uint8_t>(world_->arm_count());
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
 const double arm_q =
 quantum_rad > 0.0
 ? std::round(world_->arm_angle(i) / quantum_rad) * quantum_rad
 : world_->arm_angle(i);
    if (have_prev_ && dt_s > 0.0) {
      prev_drive_vel_radps_[i] =
          (drive_q - prev_drive_angle_rad_[i]) / dt_s;
      prev_steer_vel_radps_[i] =
          (steer_q - prev_steer_angle_rad_[i]) / dt_s;
 prev_arm_vel_radps_[i] =
 (arm_q - prev_arm_angle_rad_[i]) / dt_s;
    } else if (!have_prev_) {
      prev_drive_vel_radps_[i] = 0.0;  // just powered: no history yet
      prev_steer_vel_radps_[i] = 0.0;
 prev_arm_vel_radps_[i] = 0.0;
    }
    // Repeated stamp: hold the previous velocity (never divide by zero).
    prev_drive_angle_rad_[i] = drive_q;
    prev_steer_angle_rad_[i] = steer_q;
 prev_arm_angle_rad_[i] = arm_q;
    out.drive_radps[i] = 0.0;
    out.steer_rad[i] = 0.0;
    out.steer_vel_radps[i] = 0.0;
 out.arm_rad[i] = 0.0;
 out.arm_vel_radps[i] = 0.0;
  }
  for (int i = 0; i < out.drive_count; ++i) {
    out.drive_radps[i] = prev_drive_vel_radps_[i];
  }
  for (int i = 0; i < out.steer_count; ++i) {
    out.steer_rad[i] = prev_steer_angle_rad_[i];
    out.steer_vel_radps[i] = prev_steer_vel_radps_[i];
  }
 for (int i = 0; i < out.arm_count; ++i) {
 out.arm_rad[i] = prev_arm_angle_rad_[i];
 out.arm_vel_radps[i] = prev_arm_vel_radps_[i];
 }
  // Body twist from MEASURED joints (native-driver parity): the ROBOT
  // never sees truth body rates here. Wheel-derived through the same
  // geometry the plant was built with, so slip shows up downstream as
  // wheel-vs-IMU disagreement instead of being laundered through truth.
  // First poll (or repeated stamp) reports zero, like power-on.
  {
    const double r =
        std::max(1e-6, world_->motor_params().wheel_radius_m);
    const double track =
        std::max(0.1, world_->motor_params().track_width_m);
    const double base =
        std::max(0.1, world_->motor_params().wheel_base_m);
    const double* w = out.drive_radps.data();
 double fwd = 0.0, lat = 0.0, yaw = 0.0;
    if (out.drive_count == 2 && out.steer_count == 0) {
      // Differential: idx0 left (+y), idx1 right.
      fwd = 0.5 * (w[0] + w[1]) * r;
      yaw = (w[1] - w[0]) * r / track;
    } else if (out.drive_count == 2 && out.steer_count == 2) {
      // Ackermann: rear drive axle + bicycle yaw from measured steer.
      fwd = 0.5 * (w[0] + w[1]) * r;
 // The two front encoders report the inner and outer wheel angles.  The
 // curvature of an Ackermann axle is the mean of their individual tangent
 // curvatures; tan(mean(angle)) is only an approximation and biases yaw at
 // larger steering angles.  Use the same geometry as the chassis command
 // conversion so wheel odometry and the physical steering linkage agree.
 yaw = fwd * 0.5 *
 (std::tan(out.steer_rad[0]) + std::tan(out.steer_rad[1])) / base;
    } else if (out.drive_count == 4 && out.steer_count == 4) {
      // Swerve: rigid-body fit over measured module vectors (speed along
      // the measured steer angle at each corner module position,
      // translation removed before fitting yaw).
      const double sx[4] = {base / 2, base / 2, -base / 2, -base / 2};
      const double sy[4] = {track / 2, -track / 2, track / 2, -track / 2};
      double vx = 0.0, vy = 0.0;
      for (int i = 0; i < 4; ++i) {
        vx += w[i] * r * std::cos(out.steer_rad[i]);
        vy += w[i] * r * std::sin(out.steer_rad[i]);
      }
      vx /= 4.0;
      vy /= 4.0;
      fwd = vx;
 lat = vy;
      double num = 0.0, den = 0.0;
      for (int i = 0; i < 4; ++i) {
        const double mvx =
            w[i] * r * std::cos(out.steer_rad[i]) - vx;
        const double mvy =
            w[i] * r * std::sin(out.steer_rad[i]) - vy;
        num += sx[i] * mvy - sy[i] * mvx;
        den += sx[i] * sx[i] + sy[i] * sy[i];
      }
      yaw = den > 1e-9 ? num / den : 0.0;
    } else if (out.drive_count == 4 && out.steer_count == 0) {
      // Mecanum: roller-map rows dual to the engine wrench (same
      // coefficients as the plant's virtual-work map, power-consistent).
 const double arm = (track + base) / 2.0;
      fwd = 0.25 * (w[0] + w[1] + w[2] + w[3]) * r;
 lat = 0.25 * (-w[0] + w[1] + w[2] - w[3]) * r;
      yaw = arm > 1e-9
                ? 0.25 * (-w[0] + w[1] - w[2] + w[3]) * r / arm
                : 0.0;
    }
    out.forward_mps = (have_prev_ && dt_s > 0.0) ? fwd : 0.0;
 out.lateral_mps = (have_prev_ && dt_s > 0.0) ? lat : 0.0;
    out.yaw_rate_radps = (have_prev_ && dt_s > 0.0) ? yaw : 0.0;
  }
  have_prev_ = true;
  prev_stamp_ns_ = out.stamp_ns;
  return out.valid();
}

}  // namespace lost_signal::sim::providers
