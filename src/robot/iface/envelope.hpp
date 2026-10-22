#pragma once

#include "robot/iface/message.hpp"
#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::iface {

// Canonical pack helpers: typed provider messages -> one LSRI envelope.
// ROBOT-owned pure functions, so every suite packs identically and the rule
// is unit-tested once instead of hand-rolled per harness.
//
// v1 provenance note: the envelope carries ONE transport identity (the motion
// endpoint's). Section ownership is enforced at suite composition: a suite
// only packs messages from providers it owns. Per-section provenance is
// future work, not a second envelope.
inline ObservationFrame make_envelope(const MotionFeedback& fb) {
  ObservationFrame o;
  o.sequence = fb.sequence;
  o.stamp_ns = fb.stamp_ns;
  o.source = fb.source;
  o.frame = fb.frame;
  o.odometry_valid = fb.odometry_valid;
  o.forward_mps = fb.forward_mps;
  o.yaw_rate_radps = fb.yaw_rate_radps;
  o.drive_count = fb.drive_count;
  o.steer_count = fb.steer_count;
  o.drive_radps = fb.drive_radps;
  o.steer_rad = fb.steer_rad;
  o.steer_vel_radps = fb.steer_vel_radps;
  return o;
}

inline bool set_scan(ObservationFrame& o, const Lidar2DScan& scan) {
  if (!scan.valid()) return false;
  o.ranges_m = scan.ranges_m;
  o.scan_valid = true;
  o.scan_angle_min_rad = scan.angle_min_rad;
  o.scan_angle_increment_rad = scan.angle_increment_rad;
  return true;
}

inline bool set_imu(ObservationFrame& o, const ImuMessage& imu) {
  if (!imu.valid()) return false;
  o.imu_valid = true;
  o.imu_gyro_x_radps = imu.gyro_x_radps;
  o.imu_gyro_y_radps = imu.gyro_y_radps;
  o.imu_gyro_z_radps = imu.gyro_z_radps;
  o.imu_ax_mps2 = imu.accel_x_mps2;
  o.imu_ay_mps2 = imu.accel_y_mps2;
  o.imu_az_mps2 = imu.accel_z_mps2;
  return true;
}

inline bool set_joints(ObservationFrame& o, const MotionFeedback& fb) {
  if (!fb.has_joints()) return false;
  o.drive_count = fb.drive_count;
  o.steer_count = fb.steer_count;
  o.drive_radps = fb.drive_radps;
  o.steer_rad = fb.steer_rad;
  o.steer_vel_radps = fb.steer_vel_radps;
  return true;
}

}  // namespace lost_signal::robot::iface
