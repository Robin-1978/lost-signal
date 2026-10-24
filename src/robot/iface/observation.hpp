#pragma once

#include <array>
#include <cmath>
#include <cstddef>
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
  // Scan layout: angle_min + i * increment covers the full rotation.
  double scan_angle_min_rad = -3.141592653589793;
  double scan_angle_increment_rad = 0.0;
  // Return strength per beam, parallel to ranges_m when intensity_valid.
  // Sensing metadata only: the ROBOT never maps it back to materials.
  std::vector<float> intensities;
  bool intensity_valid = false;
  // Per-beam acquisition offsets (s, <= 0, scan-end frame), parallel to
  // ranges_m when time_offset_valid. Empty/absent = frozen scan (legacy):
  // consumers read every beam at the stamp pose.
  std::vector<float> time_offsets_s;
  bool time_offset_valid = false;

  // Static-reference scan: expected returns from the static world (map
  // raycast or survey), same ordering as ranges_m. Lets perception split
  // foreground from background. Absent for providers that cannot produce
  // it (scan_static_valid=false): classification is then unavailable while
  // raw safety coverage is retained.
  std::vector<float> static_ranges_m;
  bool scan_static_valid = false;

  // Body-frame planar odometry twist (measured, SI).
  bool odometry_valid = false;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;

  // IMU (measured, SI, full 6-axis in imu_link).
  bool imu_valid = false;
  double imu_gyro_x_radps = 0.0;
  double imu_gyro_y_radps = 0.0;
  double imu_gyro_z_radps = 0.0;
  double imu_ax_mps2 = 0.0;
  double imu_ay_mps2 = 0.0;
  double imu_az_mps2 = 0.0;

  // Joint truth (measured joints, per chassis layout).
  // Wheel controllers close their loops on this; body twist above stays
  // the convenience projection a driver derives from the same joints.
  // drive_count == 0 means absent; counts follow the actuator mode
  // (diff 2/0, ackermann 2/2, swerve 4/4, mecanum 4/0).
  std::uint8_t drive_count = 0;
  std::uint8_t steer_count = 0;
  std::array<double, 4> drive_radps = {};
  std::array<double, 4> steer_rad = {};
  std::array<double, 4> steer_vel_radps = {};

  bool has_joints() const { return drive_count > 0; }

  bool empty() const {
    return !scan_valid && !odometry_valid && !imu_valid && drive_count == 0;
  }
};

// Angle of beam i in the sensor frame. An unset increment (0) keeps the
// legacy full-rotation layout for providers that predate explicit layout;
// every consumer uses this one definition so a frame cannot be interpreted
// two different ways.
inline double scan_angle_increment_rad(const ObservationFrame& obs) {
 const double inc = obs.scan_angle_increment_rad;
 if (std::isfinite(inc) && inc != 0.0) return inc;
 const std::size_t n = obs.ranges_m.size();
 return n > 0 ? 2.0 * 3.14159265358979323846 /
 static_cast<double>(n) : 0.0;
}

inline double scan_beam_angle_rad(const ObservationFrame& obs,
 std::size_t i) {
 const double inc = scan_angle_increment_rad(obs);
 if (std::isfinite(inc) && inc != 0.0) {
 return obs.scan_angle_min_rad + inc * static_cast<double>(i);
 }
 const std::size_t n = obs.ranges_m.size();
 const double count = static_cast<double>(n > 0 ? n : 1);
 return obs.scan_angle_min_rad +
 2.0 * 3.141592653589793 * static_cast<double>(i) / count;
}

bool valid_observation_frame(const ObservationFrame& frame);

}  // namespace lost_signal::robot::iface
