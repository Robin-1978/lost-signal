#include <cassert>
#include <cmath>
#include <fstream>
#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include "robot/platform/calibration.hpp"
#include "robot/platform/description.hpp"

namespace iface = lost_signal::robot::iface;

int main() {
 using namespace lost_signal::robot::platform;
 using namespace lost_signal::robot::iface;
 using lost_signal::robot::core::Json;
 const auto description = default_robot_description();
 OfflineCalibrationConfig config;
 config.robot_id = description.robot_id;
 config.robot_description_hash = hash_robot_description(description);
 config.profile_revision = 7;
 config.valid_temperature_min_c = 0.0;
 config.valid_temperature_max_c = 45.0;
 config.valid_load_min_kg = 5.0;
 config.valid_load_max_kg = 10.0;
 config.created_unix_s = 1000.0;

 std::vector<CalibrationMeasurement> samples;
 for (int index = 0; index < 5; ++index) {
 CalibrationMeasurement sample;
 sample.timestamp_s = index;
 sample.wheel_distance_m = 10.0 + 0.1 * index;
 sample.reference_distance_m = sample.wheel_distance_m * 1.05;
 sample.stationary = index < 3;
 sample.accel_mps2 = {0.11, -0.02, 9.82};
 sample.gyro_radps = {0.001, -0.002, 0.003};
 samples.push_back(sample);
 }
 // One bad run is tolerated by the median fit only when it is not allowed to
 // dominate the evidence. It remains inside the declared profile range here.
 samples.push_back({5.0, 10.0, 10.8, {0.11, -0.02, 9.82},
 {0.001, -0.002, 0.003}, false});
 const auto fitted = calibrate_offline(config, samples);
 assert(fitted.accepted);
 assert(std::abs(fitted.artifact.wheel_radius_scale - 1.05) < 1e-9);
 assert(fitted.artifact.sample_count == samples.size());
 assert(fitted.artifact.valid());

 // Field logs are ordered acquisition streams. A repeated timestamp or a
 // non-finite value must be rejected before it can change fit weighting.
 auto unordered = samples;
 unordered[2].timestamp_s = unordered[1].timestamp_s;
 assert(!calibrate_offline(config, unordered).accepted);
 auto non_finite = samples;
 non_finite.back().gyro_radps[0] =
     std::numeric_limits<double>::quiet_NaN();
 assert(!calibrate_offline(config, non_finite).accepted);

 std::string error;
 const auto encoded = fitted.artifact.to_json();
 CalibrationArtifact decoded;
 assert(calibration_from_json(encoded, decoded, &error));
 assert(decoded.robot_description_hash == config.robot_description_hash);
 assert(decoded.input_digest == fitted.artifact.input_digest);
 assert(decoded.valid_load_min_kg == 5.0);
 assert(decoded.created_unix_s == 1000.0);

 const auto artifact_path = std::filesystem::temp_directory_path() /
                            "lost_signal_calibration_artifact.json";
 assert(save_calibration_artifact_atomic(artifact_path.string(),
                                         fitted.artifact, &error));
 CalibrationArtifact loaded_artifact;
 assert(load_calibration_artifact(artifact_path.string(), loaded_artifact,
                                  &error));
 assert(loaded_artifact.input_digest == fitted.artifact.input_digest);
 CalibrationProfileRequirements requirements;
 requirements.robot_id = config.robot_id;
 requirements.robot_description_hash = config.robot_description_hash;
 requirements.profile_revision = config.profile_revision;
 requirements.temperature_c = 25.0;
 requirements.load_kg = 7.0;
 requirements.now_unix_s = 1100.0;
 requirements.maximum_age_s = 200.0;

 iface::ChassisCapabilities chassis;
 chassis.control_owner = iface::MotionControlOwner::kHostMotionLoop;
 chassis.chassis_mode = iface::ActuatorMode::kDiffDuty;
 chassis.drive_count = 2;
 chassis.control_period_ns = 20'000'000;
 chassis.feedback_period_ns = 20'000'000;
 chassis.maximum_command_age_ns = 300'000'000;
 chassis.profile_id = "ro1.native";
 chassis.profile_revision = config.profile_revision;
 NativeHardwareProfile native_profile{chassis, fitted.artifact};
 NativeHardwareProfileRequirements native_requirements;
 native_requirements.calibration = requirements;
 native_requirements.profile_id = chassis.profile_id;
 native_requirements.control_owner = chassis.control_owner;
 native_requirements.chassis_mode = chassis.chassis_mode;
 native_requirements.drive_count = chassis.drive_count;
 native_requirements.steer_count = chassis.steer_count;
 assert(validate_native_hardware_profile(native_profile, native_requirements,
                                         &error));
 native_requirements.drive_count = 4;
 assert(!validate_native_hardware_profile(native_profile, native_requirements,
                                          &error));
 native_requirements.drive_count = chassis.drive_count;
 native_profile.chassis.profile_revision = config.profile_revision + 1;
 assert(!validate_native_hardware_profile(native_profile, native_requirements,
                                          &error));
 native_profile.chassis.profile_revision = config.profile_revision;

 // A first-run adapter can derive a deterministic commissioning profile from
 // the robot description and board handshake. It is rejected by the measured
 // gate until the adapter explicitly opts into commissioning mode.
 NativeHardwareProfile nominal;
 iface::ChassisCapabilities nominal_chassis = chassis;
 nominal_chassis.profile_id = "ro1.nominal";
 assert(make_nominal_native_hardware_profile(description, nominal_chassis,
                                             nominal, &error));
 assert(nominal.source == NativeHardwareProfileSource::kNominalCommissioning);
 assert(nominal.calibration.wheel_radius_scale == 1.0);
 assert(nominal.calibration.track_width_scale == 1.0);
 assert((nominal.calibration.imu_accel_bias_mps2 ==
         std::array<double, 3>{0.0, 0.0, 0.0}));
 assert((nominal.calibration.imu_gyro_bias_radps ==
         std::array<double, 3>{0.0, 0.0, 0.0}));
 assert(nominal.calibration.lidar_time_offset_s == 0.0);
 assert(nominal.calibration.sample_count == 0);
 assert(nominal.calibration.input_digest.rfind("nominal:", 0) == 0);
 NativeHardwareProfileRequirements nominal_requirements;
 nominal_requirements.calibration.robot_id = description.robot_id;
 nominal_requirements.calibration.robot_description_hash =
     hash_robot_description(description);
 nominal_requirements.calibration.profile_revision =
     nominal_chassis.profile_revision;
 nominal_requirements.profile_id = nominal_chassis.profile_id;
 nominal_requirements.control_owner = nominal_chassis.control_owner;
 nominal_requirements.chassis_mode = nominal_chassis.chassis_mode;
 nominal_requirements.drive_count = nominal_chassis.drive_count;
 nominal_requirements.steer_count = nominal_chassis.steer_count;
 assert(!validate_native_hardware_profile(nominal, nominal_requirements,
                                          &error));
 nominal_requirements.allow_nominal_commissioning = true;
 assert(validate_native_hardware_profile(nominal, nominal_requirements,
                                         &error));
 CalibrationArtifact nominal_with_samples = nominal.calibration;
 nominal_with_samples.sample_count = 1;
 assert(!nominal_with_samples.valid(&error));
 // Provenance is bidirectional: an explicit commissioning switch cannot
 // relabel measured evidence, and a nominal source cannot carry samples.
 NativeHardwareProfile relabeled = nominal;
 relabeled.source = NativeHardwareProfileSource::kMeasured;
 assert(!validate_native_hardware_profile(relabeled, nominal_requirements,
                                          &error));
 relabeled = nominal;
 relabeled.calibration = fitted.artifact;
 assert(!validate_native_hardware_profile(relabeled, nominal_requirements,
                                          &error));

 CalibrationArtifact accepted_profile;
 assert(load_calibration_profile(artifact_path.string(), requirements,
                                 accepted_profile, &error));
 requirements.profile_revision = 8;
 assert(!load_calibration_profile(artifact_path.string(), requirements,
                                  accepted_profile, &error));
 requirements.profile_revision = config.profile_revision;
 requirements.now_unix_s = 1301.0;
 assert(!load_calibration_profile(artifact_path.string(), requirements,
                                  accepted_profile, &error));
 std::filesystem::remove(artifact_path);

 CalibrationArtifact invalid = decoded;
 invalid.wheel_radius_scale = 1.5;
 assert(!invalid.valid(&error));
 assert(!calibration_from_json(Json::Obj({
 {"format", Json::Str("lost-signal-calibration/1")}}), decoded,
 &error));

 OnlineCalibrator online;
 online.seed(fitted.artifact);
 online.checkpoint();
 for (std::size_t index = 0; index < 5; ++index) {
 assert(!online.update_wheel_scale(0.3, 0.5) || index == 4);
 }
 assert(online.accepted_updates() == 1);
 assert(online.artifact().wheel_radius_scale <= 1.06);
 const double updated = online.artifact().wheel_radius_scale;
 online.rollback();
 assert(std::abs(online.artifact().wheel_radius_scale - 1.05) < 1e-9);
 assert(updated != online.artifact().wheel_radius_scale);
 assert(!online.update_wheel_scale(0.5, 0.5));
 assert(!online.update_wheel_scale(-0.1, 0.5));

 const auto csv = std::filesystem::temp_directory_path() /
                  "lost_signal_calibration_measurements.csv";
 {
  std::ofstream output(csv);
  assert(output.is_open());
  output << "timestamp_s,wheel_distance_m,reference_distance_m,"
            "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,stationary\n";
  output << "0,10,10.5,0.1,-0.02,9.82,0.001,-0.002,0.003,true\n";
  output << "1,10.1,10.605,0.1,-0.02,9.82,0.001,-0.002,0.003,0\n";
 }
 std::vector<CalibrationMeasurement> loaded;
 assert(load_calibration_measurements_csv(csv.string(), loaded, &error));
 assert(loaded.size() == 2);
 assert(loaded.front().stationary);
 assert(!loaded.back().stationary);
 std::filesystem::remove(csv);

 const auto bad_csv = std::filesystem::temp_directory_path() /
                      "lost_signal_bad_calibration_measurements.csv";
 {
  std::ofstream output(bad_csv);
  assert(output.is_open());
  output << "0,10,10.5,0,0,9.8,0,0,0,maybe\n";
 }
 assert(!load_calibration_measurements_csv(bad_csv.string(), loaded, &error));
 assert(!error.empty());
 std::filesystem::remove(bad_csv);

 const auto unordered_csv = std::filesystem::temp_directory_path() /
                            "lost_signal_unordered_calibration.csv";
 {
  std::ofstream output(unordered_csv);
  assert(output.is_open());
  output << "0,10,10.5,0,0,9.8,0,0,0,1\n";
  output << "0,10.1,10.605,0,0,9.8,0,0,0,0\n";
 }
 assert(!load_calibration_measurements_csv(unordered_csv.string(), loaded,
                                           &error));
 assert(!error.empty());
 std::filesystem::remove(unordered_csv);

 const auto journal_path = std::filesystem::temp_directory_path() /
                           "lost_signal_calibration_hil.jsonl";
 CalibrationJournalEntry first_entry;
 first_entry.sequence = 1;
 first_entry.timestamp_s = 10.0;
 first_entry.artifact_digest = fitted.artifact.input_digest;
 first_entry.event = "online_update";
 first_entry.temperature_c = 22.0;
 first_entry.load_kg = 7.0;
 first_entry.wheel_radius_scale = fitted.artifact.wheel_radius_scale;
 first_entry.residual = 0.2;
 first_entry.accepted = true;
 first_entry.diagnostic = "confirmed";
 assert(append_calibration_journal(journal_path.string(), first_entry, &error));
 auto second_entry = first_entry;
 second_entry.sequence = 2;
 second_entry.accepted = false;
 second_entry.diagnostic = "rollback";
 assert(append_calibration_journal(journal_path.string(), second_entry, &error));
 std::vector<CalibrationJournalEntry> journal;
 assert(load_calibration_journal(journal_path.string(), journal, &error));
 assert(journal.size() == 2 && journal.front().accepted &&
        !journal.back().accepted);
 std::filesystem::remove(journal_path);
 return 0;
}
