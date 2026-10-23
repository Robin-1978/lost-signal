#pragma once

// SIM-owned lidar device model (observation parameters, never physics).
// Geometry backends return ideal ranges; this layer applies what the device
// does to a real return: per-beam white noise + bias, range quantization,
// random dropout, and the optical near-field (min range). A measurement
// driven outside [min_range, max_range) is a no-return (miss), never a
// value clamped to the boundary — clamping would invent an obstacle at the
// range edge. The default model is exact (all zeros): legacy behavior
// bit-identical. Seeded and deterministic per provider instance.

#include <cmath>
#include <cstdint>

namespace lost_signal::sim::providers {

struct LidarDeviceModel {
  double range_stddev_m = 0.0;
  double range_bias_m = 0.0;
  double range_resolution_m = 0.0;  // 0 = no quantization
  double dropout_probability = 0.0;
  double min_range_m = 0.0;  // 0 = no near-field blanking (legacy)
};

// Fail-closed validation: stddev/resolution/min-range non-negative,
// dropout in [0, 1]. Bias is any finite value.
inline bool valid_device_model(const LidarDeviceModel& m) {
  return std::isfinite(m.range_stddev_m) && m.range_stddev_m >= 0.0 &&
         std::isfinite(m.range_bias_m) &&
         std::isfinite(m.range_resolution_m) && m.range_resolution_m >= 0.0 &&
         std::isfinite(m.dropout_probability) &&
         m.dropout_probability >= 0.0 && m.dropout_probability <= 1.0 &&
         std::isfinite(m.min_range_m) && m.min_range_m >= 0.0;
}

// Deterministic xorshift64* + Box-Muller gaussian, SIM-owned (no cross-tree
// dependency). One instance per provider; reseeding reproduces the stream.
class DeviceRng {
 public:
  explicit DeviceRng(std::uint64_t seed = 0x9E3779B97F4A7C15ULL) {
    reseed(seed);
  }
  void reseed(std::uint64_t seed) {
    state_ = seed != 0 ? seed : 0x9E3779B97F4A7C15ULL;
    spare_ready_ = false;
  }
  double uniform01() {
    state_ ^= state_ >> 12;
    state_ ^= state_ << 25;
    state_ ^= state_ >> 27;
    return (state_ * 0x2545F4914F6CDD1DULL >> 11) * (1.0 / 9007199254740992.0);
  }
  double gauss(double stddev_m) {
    if (!(stddev_m > 0.0)) return 0.0;
    if (spare_ready_) {
      spare_ready_ = false;
      return spare_ * stddev_m;
    }
    double u = 0.0, v = 0.0;
    do {
      u = uniform01();
    } while (!(u > 0.0));
    v = uniform01();
    const double r = std::sqrt(-2.0 * std::log(u));
    spare_ = r * std::sin(2.0 * 3.141592653589793 * v);
    spare_ready_ = true;
    return r * std::cos(2.0 * 3.141592653589793 * v) * stddev_m;
  }

 private:
  std::uint64_t state_ = 0x9E3779B97F4A7C15ULL;
  double spare_ = 0.0;
  bool spare_ready_ = false;
};

// One ideal HIT range through the device. Returns true with measured_m set
// (a valid return), or false for no-return (dropout or out of interval —
// the caller publishes miss/max-range, never a clamped value). Misses never
// reach this function: absent returns stay absent.
inline bool device_range(double ideal_m, double max_range_m,
                         const LidarDeviceModel& model, DeviceRng& rng,
                         double& measured_m) {
  if (!(ideal_m >= 0.0) || !(ideal_m < max_range_m)) return false;
  if (model.dropout_probability > 0.0 &&
      rng.uniform01() < model.dropout_probability) {
    return false;
  }
  double m = ideal_m + model.range_bias_m + rng.gauss(model.range_stddev_m);
  if (model.range_resolution_m > 0.0) {
    m = std::round(m / model.range_resolution_m) * model.range_resolution_m;
  }
  if (!(m >= model.min_range_m) || !(m < max_range_m)) return false;
  measured_m = m;
  return true;
}

}  // namespace lost_signal::sim::providers
