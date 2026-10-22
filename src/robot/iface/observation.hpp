#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot/core/frames.hpp"
#include "robot/core/time.hpp"

namespace lost_signal::robot::iface {

// One provider-neutral sensor snapshot. A provider fills only the fields its
// endpoint owns; ROBOT treats missing/stale required data as fail-closed.
struct ObservationFrame {
  std::uint64_t sequence = 0;
  core::MonotonicNs stamp_ns = core::kInvalidTime;  // acquisition time
  std::string source;                               // stable endpoint id
  core::Frame frame = core::Frame::kUnknown;

  // Minimal planar scan. Empty + scan_valid=false means unknown, never free.
  std::vector<float> ranges_m;
  bool scan_valid = false;

  // Body-frame planar odometry twist (measured, SI).
  bool odometry_valid = false;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;

  // IMU (measured, SI).
  bool imu_valid = false;
  double imu_yaw_rate_radps = 0.0;
  double imu_ax_mps2 = 0.0;

  bool empty() const { return !scan_valid && !odometry_valid && !imu_valid; }
};

bool valid_observation_frame(const ObservationFrame& frame);

}  // namespace lost_signal::robot::iface
