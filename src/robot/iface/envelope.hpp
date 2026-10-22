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
  return o;
}

inline bool set_scan(ObservationFrame& o, const Lidar2DScan& scan) {
  if (!scan.valid()) return false;
  o.ranges_m = scan.ranges_m;
  o.scan_valid = true;
  return true;
}

inline bool set_imu(ObservationFrame& o, const ImuMessage& imu) {
  if (!imu.valid()) return false;
  o.imu_valid = true;
  o.imu_yaw_rate_radps = imu.yaw_rate_radps;
  o.imu_ax_mps2 = imu.ax_mps2;
  return true;
}

}  // namespace lost_signal::robot::iface
