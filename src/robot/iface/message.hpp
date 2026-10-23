#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot/core/frames.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/version.hpp"

namespace lost_signal::robot::iface {

// IMessage: typed provider message. Every sensor message implements this so
// providers, channels, and loggers share stamp/source/sequence/frame rules.
// Agnostic to transport: in-process call today, serialized wire tomorrow.
class IMessage {
 public:
  virtual ~IMessage() = default;
  virtual int schema_version() const = 0;
  virtual core::MonotonicNs stamp_ns() const = 0;
  virtual const std::string& source() const = 0;
  virtual std::uint64_t sequence() const = 0;
  virtual core::Frame frame() const = 0;
  virtual bool valid() const = 0;
};

// 2D planar scan message. Missing return = absent, never max-range fill.
// Parallel intensities carry the return strength per beam (sensing
// quantity from the provider's reflectance model; empty = unknown).
// Parallel time offsets carry each beam's acquisition time relative to
// the scan stamp (<= 0, scan-end frame; empty = frozen-scan legacy).
// The ROBOT never maps intensity back to ground truth (no material
// lookup): it rides along as measurement metadata.
struct Lidar2DScan : public IMessage {  std::uint64_t seq = 0;
  core::MonotonicNs stamp = core::kInvalidTime;
  std::string src;
  core::Frame frm = core::Frame::kLidarLink;
  std::vector<float> ranges_m;
  std::vector<float> intensities;
  std::vector<float> time_offsets_s;
  float angle_min_rad = 0.0f;
  float angle_increment_rad = 0.0f;

  int schema_version() const override { return kMessageSchemaVersion; }
  core::MonotonicNs stamp_ns() const override { return stamp; }
  const std::string& source() const override { return src; }
  std::uint64_t sequence() const override { return seq; }
  core::Frame frame() const override { return frm; }
  bool valid() const override;
};

// 3D multi-line cloud message. One rotation = one message.
struct Lidar3DPoint {
  float x_m = 0.0f;
  float y_m = 0.0f;
  float z_m = 0.0f;
  float intensity = 0.0f;
  std::uint16_t ring = 0;
  float time_offset_s = 0.0f;  // in [-scan_period, 0], relative to stamp
};

struct Lidar3DCloud : public IMessage {
  std::uint64_t seq = 0;
  core::MonotonicNs stamp = core::kInvalidTime;
  std::string src;
  core::Frame frm = core::Frame::kLidarLink;
  std::uint16_t ring_count = 0;
  float scan_period_s = 0.0f;
  std::vector<Lidar3DPoint> points;

  int schema_version() const override { return kMessageSchemaVersion; }
  core::MonotonicNs stamp_ns() const override { return stamp; }
  const std::string& source() const override { return src; }
  std::uint64_t sequence() const override { return seq; }
  core::Frame frame() const override { return frm; }
  bool valid() const override;
};

// IMU message: full 6-axis measured motion in imu_link (gyro + accel).
// Bias, noise, and mounting belong to the provider's device model;
// the estimate never sees truth.
struct ImuMessage : public IMessage {
  std::uint64_t seq = 0;
  core::MonotonicNs stamp = core::kInvalidTime;
  std::string src;
  core::Frame frm = core::Frame::kImuLink;
  double gyro_x_radps = 0.0;
  double gyro_y_radps = 0.0;
  double gyro_z_radps = 0.0;
  double accel_x_mps2 = 0.0;
  double accel_y_mps2 = 0.0;
  double accel_z_mps2 = 0.0;

  int schema_version() const override { return kMessageSchemaVersion; }
  core::MonotonicNs stamp_ns() const override { return stamp; }
  const std::string& source() const override { return src; }
  std::uint64_t sequence() const override { return seq; }
  core::Frame frame() const override { return frm; }
  bool valid() const override;
};

}  // namespace lost_signal::robot::iface
