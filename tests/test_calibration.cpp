#include <cassert>
#include <cmath>
#include <string>
#include <vector>

#include "robot/platform/calibration.hpp"
#include "robot/platform/description.hpp"

int main() {
 using namespace lost_signal::robot::platform;
 using lost_signal::robot::core::Json;
 const auto description = default_robot_description();
 OfflineCalibrationConfig config;
 config.robot_id = description.robot_id;
 config.robot_description_hash = hash_robot_description(description);
 config.profile_revision = 7;

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

 std::string error;
 const auto encoded = fitted.artifact.to_json();
 CalibrationArtifact decoded;
 assert(calibration_from_json(encoded, decoded, &error));
 assert(decoded.robot_description_hash == config.robot_description_hash);
 assert(decoded.input_digest == fitted.artifact.input_digest);

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
 return 0;
}
