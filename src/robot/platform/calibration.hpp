#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot/core/json.hpp"
#include "robot/iface/motion.hpp"

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
 // A zero/zero load range preserves legacy artifacts that were calibrated
 // without a payload measurement. A non-zero range is mandatory when a
 // deployment asks the profile validator to check load applicability.
 double valid_load_min_kg = 0.0;
 double valid_load_max_kg = 0.0;
 // Wall-clock creation time is metadata for profile expiry only; it never
 // enters ROBOT estimation or simulation decisions.
 double created_unix_s = 0.0;

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
 double valid_temperature_min_c = -40.0;
 double valid_temperature_max_c = 85.0;
 double valid_load_min_kg = 0.0;
 double valid_load_max_kg = 0.0;
 double created_unix_s = 0.0;
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

// Deployment-side profile gate. All supplied requirements are explicit so a
// caller cannot accidentally accept an artifact for another robot, profile,
// temperature/load range or stale creation time.
struct CalibrationProfileRequirements {
 std::string robot_id;
 std::uint64_t robot_description_hash = 0;
 std::uint32_t profile_revision = 0;
 std::optional<double> temperature_c;
 std::optional<double> load_kg;
 std::optional<double> now_unix_s;
 std::optional<double> maximum_age_s;
};

bool validate_calibration_profile(const CalibrationArtifact& artifact,
                                  const CalibrationProfileRequirements& req,
                                  std::string* error = nullptr);

// Host-side deployment profile assembled from the board handshake and the
// calibration artifact. This is a validation seam for native adapters; it is
// not a new ROBOT data source and contains no simulator state.
struct NativeHardwareProfile {
 iface::ChassisCapabilities chassis;
 CalibrationArtifact calibration;
};

struct NativeHardwareProfileRequirements {
 CalibrationProfileRequirements calibration;
 std::string profile_id;
 std::optional<iface::MotionControlOwner> control_owner;
 std::optional<iface::ActuatorMode> chassis_mode;
 std::optional<std::uint8_t> drive_count;
 std::optional<std::uint8_t> steer_count;
};

bool validate_native_hardware_profile(
    const NativeHardwareProfile& profile,
    const NativeHardwareProfileRequirements& req,
    std::string* error = nullptr);

// Writes a complete artifact to a same-directory temporary file and replaces
// the destination with one rename. A truncated or malformed file is never
// published as the active calibration.
bool save_calibration_artifact_atomic(const std::string& path,
                                      const CalibrationArtifact& artifact,
                                      std::string* error = nullptr);

bool load_calibration_artifact(const std::string& path,
                               CalibrationArtifact& out,
                               std::string* error = nullptr);

bool load_calibration_profile(const std::string& path,
                             const CalibrationProfileRequirements& req,
                             CalibrationArtifact& out,
                             std::string* error = nullptr);

// Small append-only HIL audit record. It records measured operating context
// and the artifact revision used for an update; it contains no simulator
// truth, actor identity or score.
struct CalibrationJournalEntry {
 std::uint64_t sequence = 0;
 double timestamp_s = 0.0;
 std::string artifact_digest;
 std::string event;
 double temperature_c = 0.0;
 double load_kg = 0.0;
 double wheel_radius_scale = 1.0;
 double residual = 0.0;
 bool accepted = false;
 std::string diagnostic;

 bool valid(std::string* error = nullptr) const;
 core::Json to_json() const;
};

bool append_calibration_journal(const std::string& path,
                                const CalibrationJournalEntry& entry,
                                std::string* error = nullptr);

bool load_calibration_journal(const std::string& path,
                              std::vector<CalibrationJournalEntry>& out,
                              std::string* error = nullptr);

// Portable offline-record bridge. The file contains one measurement per row
// in the order documented below; a header row is optional. It is deliberately
// a small CSV reader so a field log can be calibrated without pulling a
// simulator or a third-party serialization library into ROBOT.
//
// timestamp_s,wheel_distance_m,reference_distance_m,
// accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,stationary
bool load_calibration_measurements_csv(
    const std::string& path, std::vector<CalibrationMeasurement>& out,
    std::string* error = nullptr);

} // namespace lost_signal::robot::platform
