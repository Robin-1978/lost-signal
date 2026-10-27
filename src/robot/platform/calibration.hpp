#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/core/json.hpp"

namespace lost_signal::robot::platform {

// Versioned, deployable calibration evidence. The artifact contains only
// parameters that can be estimated from measured sensors and an external
// reference; it never contains simulator truth, actor labels, or score.
struct CalibrationArtifact {
 std::string format = "lost-signal-calibration/1";
 std::string robot_id;
 std::uint64_t robot_description_hash = 0;
 std::uint32_t profile_revision = 0;
 std::string input_digest;
 double wheel_radius_scale = 1.0;
 double track_width_scale = 1.0;
 std::array<double, 3> imu_accel_bias_mps2{};
 std::array<double, 3> imu_gyro_bias_radps{};
 double lidar_time_offset_s = 0.0;
 double residual_rms = 0.0;
 std::size_t sample_count = 0;
 double valid_temperature_min_c = -40.0;
 double valid_temperature_max_c = 85.0;

 bool valid(std::string* error = nullptr) const;
 core::Json to_json() const;
};

// A recorded measurement used by the offline solver. reference_distance_m is
// an operator/mechanical reference (for example a tape-measured travel), not
// a simulator pose. Samples may be sparse and may contain stationary IMU
// windows for bias estimation.
struct CalibrationMeasurement {
 double timestamp_s = 0.0;
 double wheel_distance_m = 0.0;
 double reference_distance_m = 0.0;
 std::array<double, 3> accel_mps2{};
 std::array<double, 3> gyro_radps{};
 bool stationary = false;
};

struct OfflineCalibrationConfig {
 std::string robot_id;
 std::uint64_t robot_description_hash = 0;
 std::uint32_t profile_revision = 0;
 double minimum_reference_distance_m = 0.25;
 double maximum_scale_deviation = 0.20;
};

struct OfflineCalibrationResult {
 bool accepted = false;
 std::string diagnostic;
 CalibrationArtifact artifact;
};

// Deterministic, robust offline fit: median wheel scale and mean stationary
// IMU bias. The caller supplies the recorded measurement stream; no wall
// clock or simulation state is consulted.
OfflineCalibrationResult calibrate_offline(
 const OfflineCalibrationConfig& config,
 const std::vector<CalibrationMeasurement>& samples);

struct OnlineCalibrationConfig {
 double gain = 0.05;
 double maximum_step = 0.01;
 double minimum_residual_improvement = 1e-4;
 double minimum_scale = 0.80;
 double maximum_scale = 1.20;
 std::size_t confirmation_samples = 5;
};

// Bounded online update with a rollback checkpoint. It is deliberately
// separate from navigation: callers can apply the accepted artifact only at
// a lifecycle/profile boundary, never inside a motion tick.
class OnlineCalibrator {
 public:
 explicit OnlineCalibrator(OnlineCalibrationConfig config = {});

 void seed(const CalibrationArtifact& artifact);
 bool update_wheel_scale(double residual, double baseline_residual);
 void checkpoint();
 void rollback();
 const CalibrationArtifact& artifact() const { return artifact_; }
 std::size_t accepted_updates() const { return accepted_updates_; }

 private:
 OnlineCalibrationConfig config_;
 CalibrationArtifact artifact_;
 CalibrationArtifact checkpoint_;
 std::size_t consecutive_improvements_ = 0;
 std::size_t accepted_updates_ = 0;
};

bool calibration_from_json(const core::Json& value,
 CalibrationArtifact& out, std::string* error = nullptr);

} // namespace lost_signal::robot::platform
