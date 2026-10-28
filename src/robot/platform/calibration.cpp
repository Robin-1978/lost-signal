#include "robot/platform/calibration.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lost_signal::robot::platform {

namespace {

constexpr double kScaleFloor = 1e-9;

bool finite(double value) { return std::isfinite(value); }

void fail(std::string* error, const char* message) {
 if (error != nullptr) *error = message;
}

bool number(const core::Json& object, const char* key, double& out,
 std::string* error) {
 const core::Json* value = object.find(key);
 if (value == nullptr || !value->is_num() || !finite(value->as_num())) {
 fail(error, "calibration field must be a finite number");
 return false;
 }
 out = value->as_num();
 return true;
}

bool integer(const core::Json& object, const char* key, std::uint64_t& out,
 std::string* error) {
 const core::Json* value = object.find(key);
 if (value != nullptr && value->is_str()) {
 try {
 std::size_t consumed = 0;
 const auto parsed = std::stoull(value->as_str(), &consumed, 10);
 if (consumed == value->as_str().size()) {
 out = parsed;
 return true;
 }
 } catch (...) {
 }
 }
 if (value == nullptr || !value->is_num() || !finite(value->as_num()) ||
 value->as_num() < 0.0 || value->as_num() >
 static_cast<double>(std::numeric_limits<std::uint64_t>::max()) ||
 std::floor(value->as_num()) != value->as_num()) {
 fail(error, "calibration integer field is invalid");
 return false;
 }
 out = static_cast<std::uint64_t>(value->as_num());
 return true;
}

bool string(const core::Json& object, const char* key, std::string& out,
 std::string* error) {
 const core::Json* value = object.find(key);
 if (value == nullptr || !value->is_str() || value->as_str().empty()) {
 fail(error, "calibration string field is invalid");
 return false;
 }
 out = value->as_str();
 return true;
}

std::uint64_t fnv_append(std::uint64_t hash, std::uint64_t value) {
 for (int shift = 0; shift < 64; shift += 8) {
 hash ^= (value >> shift) & 0xffU;
 hash *= 1099511628211ULL;
 }
 return hash;
}

std::string digest(const std::vector<CalibrationMeasurement>& samples) {
 std::uint64_t hash = 1469598103934665603ULL;
 for (const auto& sample : samples) {
 const std::array<double, 9> values{
 sample.timestamp_s, sample.wheel_distance_m,
 sample.reference_distance_m, sample.accel_mps2[0], sample.accel_mps2[1],
 sample.accel_mps2[2], sample.gyro_radps[0], sample.gyro_radps[1],
 sample.gyro_radps[2]};
 for (double value : values) {
 std::uint64_t bits = 0;
 static_assert(sizeof(bits) == sizeof(value));
 std::memcpy(&bits, &value, sizeof(bits));
 hash = fnv_append(hash, bits);
 }
 hash = fnv_append(hash, sample.stationary ? 1U : 0U);
 }
 return std::to_string(hash);
}

} // namespace

bool CalibrationArtifact::valid(std::string* error) const {
 if (format != "lost-signal-calibration/1") {
 fail(error, "unsupported calibration format");
 return false;
 }
 if (robot_id.empty() || robot_description_hash == 0 || profile_revision == 0 ||
 input_digest.empty()) {
 fail(error, "calibration identity is incomplete");
 return false;
 }
 const auto finite_array = [](const auto& values) {
 return std::all_of(values.begin(), values.end(),
 [](double value) { return std::isfinite(value); });
 };
 if (!finite(wheel_radius_scale) || !finite(track_width_scale) ||
 wheel_radius_scale < 0.80 || wheel_radius_scale > 1.20 ||
 track_width_scale < 0.80 || track_width_scale > 1.20 ||
 !finite_array(imu_accel_bias_mps2) || !finite_array(imu_gyro_bias_radps) ||
 !finite(lidar_time_offset_s) || !finite(residual_rms) || residual_rms < 0.0 ||
 sample_count == 0 || !finite(valid_temperature_min_c) ||
 !finite(valid_temperature_max_c) ||
 valid_temperature_min_c >= valid_temperature_max_c) {
 fail(error, "calibration value is outside its physical bounds");
 return false;
 }
 return true;
}

core::Json CalibrationArtifact::to_json() const {
 core::Json::Array accel;
 core::Json::Array gyro;
 for (double value : imu_accel_bias_mps2) accel.push_back(core::Json::Num(value));
 for (double value : imu_gyro_bias_radps) gyro.push_back(core::Json::Num(value));
 return core::Json::Obj({
 {"format", core::Json::Str(format)},
 {"robot_id", core::Json::Str(robot_id)},
 {"robot_description_hash", core::Json::Str(
 std::to_string(robot_description_hash))},
 {"profile_revision", core::Json::Num(static_cast<double>(profile_revision))},
 {"input_digest", core::Json::Str(input_digest)},
 {"wheel_radius_scale", core::Json::Num(wheel_radius_scale)},
 {"track_width_scale", core::Json::Num(track_width_scale)},
 {"imu_accel_bias_mps2", core::Json::Arr(std::move(accel))},
 {"imu_gyro_bias_radps", core::Json::Arr(std::move(gyro))},
 {"lidar_time_offset_s", core::Json::Num(lidar_time_offset_s)},
 {"residual_rms", core::Json::Num(residual_rms)},
 {"sample_count", core::Json::Num(static_cast<double>(sample_count))},
 {"valid_temperature_min_c", core::Json::Num(valid_temperature_min_c)},
 {"valid_temperature_max_c", core::Json::Num(valid_temperature_max_c)}});
}

OfflineCalibrationResult calibrate_offline(
 const OfflineCalibrationConfig& config,
 const std::vector<CalibrationMeasurement>& samples) {
 OfflineCalibrationResult result;
 result.artifact.robot_id = config.robot_id;
 result.artifact.robot_description_hash = config.robot_description_hash;
 result.artifact.profile_revision = config.profile_revision;
 result.artifact.input_digest = digest(samples);
 if (config.robot_id.empty() || config.robot_description_hash == 0 ||
 config.profile_revision == 0 || samples.empty() ||
 !finite(config.minimum_reference_distance_m) ||
 config.minimum_reference_distance_m <= 0.0 ||
 !finite(config.maximum_scale_deviation) ||
 config.maximum_scale_deviation <= 0.0 ||
 config.maximum_scale_deviation >= 1.0) {
 result.diagnostic = "invalid offline calibration configuration";
 return result;
 }
 std::vector<double> scales;
 std::array<double, 3> accel_sum{};
 std::array<double, 3> gyro_sum{};
 std::size_t stationary_count = 0;
 for (const auto& sample : samples) {
 if (!finite(sample.timestamp_s) || !finite(sample.wheel_distance_m) ||
 !finite(sample.reference_distance_m)) {
 result.diagnostic = "non-finite calibration measurement";
 return result;
 }
 if (std::abs(sample.wheel_distance_m) >= kScaleFloor &&
 std::abs(sample.reference_distance_m) >= config.minimum_reference_distance_m) {
 const double scale = sample.reference_distance_m / sample.wheel_distance_m;
 if (finite(scale)) scales.push_back(scale);
 }
 if (sample.stationary) {
 bool valid_imu = true;
 for (std::size_t axis = 0; axis < 3; ++axis) {
 valid_imu = valid_imu && finite(sample.accel_mps2[axis]) &&
 finite(sample.gyro_radps[axis]);
 }
 if (!valid_imu) {
 result.diagnostic = "non-finite stationary IMU measurement";
 return result;
 }
 for (std::size_t axis = 0; axis < 3; ++axis) {
 accel_sum[axis] += sample.accel_mps2[axis];
 gyro_sum[axis] += sample.gyro_radps[axis];
 }
 ++stationary_count;
 }
 }
 if (scales.empty() || stationary_count == 0) {
 result.diagnostic = "insufficient distance or stationary IMU evidence";
 return result;
 }
 std::sort(scales.begin(), scales.end());
 const std::size_t middle = scales.size() / 2;
 const double scale = scales.size() % 2 == 0
 ? 0.5 * (scales[middle - 1] + scales[middle]) : scales[middle];
 const double lower = 1.0 - config.maximum_scale_deviation;
 const double upper = 1.0 + config.maximum_scale_deviation;
 if (!finite(scale) || scale < lower || scale > upper) {
 result.diagnostic = "distance scale is outside the configured profile range";
 return result;
 }
 result.artifact.wheel_radius_scale = scale;
 for (std::size_t axis = 0; axis < 3; ++axis) {
 result.artifact.imu_accel_bias_mps2[axis] = accel_sum[axis] /
 static_cast<double>(stationary_count);
 result.artifact.imu_gyro_bias_radps[axis] = gyro_sum[axis] /
 static_cast<double>(stationary_count);
 }
 double sum_squared = 0.0;
 for (double value : scales) sum_squared += (value - scale) * (value - scale);
 result.artifact.residual_rms = std::sqrt(sum_squared /
 static_cast<double>(scales.size()));
 result.artifact.sample_count = samples.size();
 result.accepted = result.artifact.valid(&result.diagnostic);
 if (result.accepted) result.diagnostic = "accepted";
 return result;
}

OnlineCalibrator::OnlineCalibrator(OnlineCalibrationConfig config)
 : config_(config) {}

void OnlineCalibrator::seed(const CalibrationArtifact& artifact) {
 if (!artifact.valid()) return;
 artifact_ = artifact;
 checkpoint_ = artifact;
 consecutive_improvements_ = 0;
 accepted_updates_ = 0;
}

bool OnlineCalibrator::update_wheel_scale(double residual,
 double baseline_residual) {
 if (!finite(residual) || !finite(baseline_residual) ||
 baseline_residual - residual < config_.minimum_residual_improvement ||
 config_.confirmation_samples == 0 || !finite(config_.gain) ||
 !finite(config_.maximum_step) || config_.gain <= 0.0 ||
 config_.maximum_step <= 0.0) {
 consecutive_improvements_ = 0;
 return false;
 }
 ++consecutive_improvements_;
 if (consecutive_improvements_ < config_.confirmation_samples) return false;
 const double correction = std::clamp(config_.gain *
 (baseline_residual - residual), -config_.maximum_step,
 config_.maximum_step);
 const double next = std::clamp(artifact_.wheel_radius_scale + correction,
 config_.minimum_scale, config_.maximum_scale);
 if (!finite(next) || next == artifact_.wheel_radius_scale) {
 consecutive_improvements_ = 0;
 return false;
 }
 artifact_.wheel_radius_scale = next;
 artifact_.residual_rms = residual;
 ++accepted_updates_;
 consecutive_improvements_ = 0;
 return true;
}

void OnlineCalibrator::checkpoint() { checkpoint_ = artifact_; }

void OnlineCalibrator::rollback() {
 if (checkpoint_.valid()) artifact_ = checkpoint_;
 consecutive_improvements_ = 0;
}

bool calibration_from_json(const core::Json& value,
 CalibrationArtifact& out, std::string* error) {
 if (!value.is_object()) {
 fail(error, "calibration artifact must be an object");
 return false;
 }
 CalibrationArtifact parsed;
 if (!string(value, "format", parsed.format, error) ||
 !string(value, "robot_id", parsed.robot_id, error) ||
 !string(value, "input_digest", parsed.input_digest, error)) return false;
 std::uint64_t integer_value = 0;
 if (!integer(value, "robot_description_hash", parsed.robot_description_hash,
 error) || !integer(value, "profile_revision", integer_value, error)) {
 return false;
 }
 parsed.profile_revision = static_cast<std::uint32_t>(integer_value);
 if (!number(value, "wheel_radius_scale", parsed.wheel_radius_scale, error) ||
 !number(value, "track_width_scale", parsed.track_width_scale, error) ||
 !number(value, "lidar_time_offset_s", parsed.lidar_time_offset_s, error) ||
 !number(value, "residual_rms", parsed.residual_rms, error) ||
 !number(value, "valid_temperature_min_c", parsed.valid_temperature_min_c,
 error) || !number(value, "valid_temperature_max_c",
 parsed.valid_temperature_max_c, error)) return false;
 if (!integer(value, "sample_count", integer_value, error)) return false;
 parsed.sample_count = static_cast<std::size_t>(integer_value);
 const auto parse_array = [&](const char* key, auto& destination) {
 const core::Json* array = value.find(key);
 if (array == nullptr || !array->is_array() ||
 array->as_array().size() != destination.size()) return false;
 for (std::size_t index = 0; index < destination.size(); ++index) {
 if (!array->as_array()[index].is_num() ||
 !finite(array->as_array()[index].as_num())) return false;
 destination[index] = array->as_array()[index].as_num();
 }
 return true;
 };
 if (!parse_array("imu_accel_bias_mps2", parsed.imu_accel_bias_mps2) ||
 !parse_array("imu_gyro_bias_radps", parsed.imu_gyro_bias_radps) ||
 !parsed.valid(error)) {
 if (error != nullptr && error->empty()) *error = "invalid calibration arrays";
 return false;
 }
 out = std::move(parsed);
 return true;
}

bool load_calibration_measurements_csv(
    const std::string& path, std::vector<CalibrationMeasurement>& out,
    std::string* error) {
  std::ifstream input(path);
  if (!input.is_open()) {
    fail(error, "cannot open calibration CSV");
    return false;
  }
  std::vector<CalibrationMeasurement> parsed;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input, line)) {
    ++line_number;
    const auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') continue;
    std::vector<std::string> fields;
    std::stringstream row(line.substr(first));
    std::string field;
    while (std::getline(row, field, ',')) {
      const auto begin = field.find_first_not_of(" \t\r");
      const auto end = field.find_last_not_of(" \t\r");
      fields.push_back(begin == std::string::npos
                          ? std::string{}
                          : field.substr(begin, end - begin + 1));
    }
    if (fields.size() == 10 && fields[0] == "timestamp_s") continue;
    if (fields.size() != 10) {
      if (error != nullptr) {
        *error = "calibration CSV line " + std::to_string(line_number) +
                 " must contain 10 fields";
      }
      return false;
    }
    CalibrationMeasurement sample;
    double* values[] = {
        &sample.timestamp_s, &sample.wheel_distance_m,
        &sample.reference_distance_m, &sample.accel_mps2[0],
        &sample.accel_mps2[1], &sample.accel_mps2[2],
        &sample.gyro_radps[0], &sample.gyro_radps[1],
        &sample.gyro_radps[2]};
    for (std::size_t index = 0; index < 9; ++index) {
      try {
        std::size_t consumed = 0;
        *values[index] = std::stod(fields[index], &consumed);
        if (consumed != fields[index].size() ||
            !std::isfinite(*values[index])) throw std::invalid_argument("number");
      } catch (...) {
        if (error != nullptr) {
          *error = "calibration CSV line " + std::to_string(line_number) +
                   " has an invalid numeric field";
        }
        return false;
      }
    }
    if (fields[9] == "1" || fields[9] == "true") {
      sample.stationary = true;
    } else if (fields[9] == "0" || fields[9] == "false") {
      sample.stationary = false;
    } else {
      if (error != nullptr) {
        *error = "calibration CSV line " + std::to_string(line_number) +
                 " has an invalid stationary flag";
      }
      return false;
    }
    parsed.push_back(sample);
  }
  if (parsed.empty()) {
    fail(error, "calibration CSV contains no measurements");
    return false;
  }
  out = std::move(parsed);
  return true;
}

} // namespace lost_signal::robot::platform
