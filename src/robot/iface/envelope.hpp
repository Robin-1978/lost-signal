#pragma once

#include "robot/iface/message.hpp"
#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::iface {

// Canonical pack helpers: typed provider messages -> one LSRI envelope.
// ROBOT-owned pure functions, so every suite packs identically and the rule
// is unit-tested once instead of hand-rolled per harness.
//
// The envelope has one transport identity, while every populated section
// retains the typed provider's acquisition header. Algorithms use section
// time; the transport header is only for queue ordering.
inline ObservationFrame make_envelope(const MotionFeedback& fb) {
  ObservationFrame o;
  o.sequence = fb.sequence;
  o.stamp_ns = fb.stamp_ns;
  o.source = fb.source;
  o.frame = fb.frame;
 o.motion_header = {fb.sequence, fb.stamp_ns, fb.source, fb.frame};
  o.odometry_valid = fb.odometry_valid;
  o.forward_mps = fb.forward_mps;
 o.lateral_mps = fb.lateral_mps;
  o.yaw_rate_radps = fb.yaw_rate_radps;
  o.drive_count = fb.drive_count;
  o.steer_count = fb.steer_count;
  o.drive_radps = fb.drive_radps;
  o.steer_rad = fb.steer_rad;
  o.steer_vel_radps = fb.steer_vel_radps;
 o.arm_count = fb.arm_count;
 o.arm_rad = fb.arm_rad;
 o.arm_vel_radps = fb.arm_vel_radps;
  return o;
}

inline bool set_scan(ObservationFrame& o, const Lidar2DScan& scan) {
  if (!scan.valid()) return false;
 o.scan_header = {scan.seq, scan.stamp, scan.src, scan.frm};
  o.ranges_m = scan.ranges_m;
  o.scan_valid = true;
  o.scan_angle_min_rad = scan.angle_min_rad;
  o.scan_angle_increment_rad = scan.angle_increment_rad;
  // Intensities ride parallel when the provider synthesized them; absent
  // (legacy/unknown) leaves the flag false, never a zero-filled guess.
  if (!scan.intensities.empty() &&
      scan.intensities.size() == scan.ranges_m.size()) {
    o.intensities = scan.intensities;
    o.intensity_valid = true;
  } else {
    o.intensities.clear();
    o.intensity_valid = false;
  }
  // Time offsets ride parallel when the provider scanned over time; absent
  // (frozen legacy) leaves consumers at the stamp pose, never a guess.
  if (!scan.time_offsets_s.empty() &&
      scan.time_offsets_s.size() == scan.ranges_m.size()) {
    o.time_offsets_s = scan.time_offsets_s;
    o.time_offset_valid = true;
  } else {
    o.time_offsets_s.clear();
    o.time_offset_valid = false;
  }
  return true;
}

inline bool set_imu(ObservationFrame& o, const ImuMessage& imu) {
  if (!imu.valid()) return false;
 o.imu_header = {imu.seq, imu.stamp, imu.src, imu.frm};
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
 o.motion_header = {fb.sequence, fb.stamp_ns, fb.source, fb.frame};
  o.drive_count = fb.drive_count;
  o.steer_count = fb.steer_count;
  o.drive_radps = fb.drive_radps;
  o.steer_rad = fb.steer_rad;
  o.steer_vel_radps = fb.steer_vel_radps;
 o.arm_count = fb.arm_count;
 o.arm_rad = fb.arm_rad;
 o.arm_vel_radps = fb.arm_vel_radps;
  return true;
}

}  // namespace lost_signal::robot::iface
