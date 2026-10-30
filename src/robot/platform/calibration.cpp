#include "robot/platform/calibration.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace lost_signal::robot::platform {

namespace {

constexpr double kScaleFloor = 1e-9;
constexpr char kNominalDigestPrefix[] = "nominal:";

bool finite(double value) { return std::isfinite(value); }

bool nominal_digest(const std::string& value) {
 return value.rfind(kNominalDigestPrefix, 0) == 0;
}

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

bool valid_measurement_sample(const CalibrationMeasurement& sample,
                              const CalibrationMeasurement* previous) {
 if (!finite(sample.timestamp_s) || sample.timestamp_s < 0.0 ||
     !finite(sample.wheel_distance_m) ||
     !finite(sample.reference_distance_m)) {
  return false;
 }
 for (std::size_t axis = 0; axis < 3; ++axis) {
  if (!finite(sample.accel_mps2[axis]) ||
      !finite(sample.gyro_radps[axis])) {
   return false;
  }
 }
 // A field log is an ordered acquisition stream. Rejecting a repeated or
 // regressed stamp is safer than silently changing offline-fit weighting.
 return previous == nullptr || sample.timestamp_s > previous->timestamp_s;
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
 (sample_count == 0 && !nominal_digest(input_digest)) ||
 (sample_count != 0 && nominal_digest(input_digest)) ||
 !finite(valid_temperature_min_c) ||
 !finite(valid_temperature_max_c) ||
 valid_temperature_min_c >= valid_temperature_max_c ||
 !finite(valid_load_min_kg) || !finite(valid_load_max_kg) ||
 ((valid_load_min_kg != 0.0 || valid_load_max_kg != 0.0) &&
  (valid_load_min_kg < 0.0 || valid_load_max_kg <= valid_load_min_kg)) ||
 !finite(created_unix_s) || created_unix_s < 0.0) {
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
 {"valid_temperature_max_c", core::Json::Num(valid_temperature_max_c)},
 {"valid_load_min_kg", core::Json::Num(valid_load_min_kg)},
 {"valid_load_max_kg", core::Json::Num(valid_load_max_kg)},
 {"created_unix_s", core::Json::Num(created_unix_s)}});
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
 config.maximum_scale_deviation >= 1.0 ||
 !finite(config.valid_temperature_min_c) ||
 !finite(config.valid_temperature_max_c) ||
 config.valid_temperature_min_c >= config.valid_temperature_max_c ||
 !finite(config.valid_load_min_kg) || !finite(config.valid_load_max_kg) ||
 ((config.valid_load_min_kg != 0.0 || config.valid_load_max_kg != 0.0) &&
  (config.valid_load_min_kg < 0.0 ||
   config.valid_load_max_kg <= config.valid_load_min_kg)) ||
 !finite(config.created_unix_s) || config.created_unix_s < 0.0) {
 result.diagnostic = "invalid offline calibration configuration";
 return result;
 }
 std::vector<double> scales;
 std::array<double, 3> accel_sum{};
 std::array<double, 3> gyro_sum{};
 std::size_t stationary_count = 0;
 const CalibrationMeasurement* previous = nullptr;
 for (const auto& sample : samples) {
 if (!valid_measurement_sample(sample, previous)) {
 result.diagnostic = "calibration measurements are not finite and strictly "
                     "time ordered";
 return result;
 }
 previous = &sample;
 if (std::abs(sample.wheel_distance_m) >= kScaleFloor &&
 std::abs(sample.reference_distance_m) >= config.minimum_reference_distance_m) {
 const double scale = sample.reference_distance_m / sample.wheel_distance_m;
 if (finite(scale)) scales.push_back(scale);
 }
 if (sample.stationary) {
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
 result.artifact.valid_temperature_min_c = config.valid_temperature_min_c;
 result.artifact.valid_temperature_max_c = config.valid_temperature_max_c;
 result.artifact.valid_load_min_kg = config.valid_load_min_kg;
 result.artifact.valid_load_max_kg = config.valid_load_max_kg;
 result.artifact.created_unix_s = config.created_unix_s;
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
 residual < 0.0 || baseline_residual < 0.0 ||
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
 if (integer_value > std::numeric_limits<std::uint32_t>::max()) {
  fail(error, "calibration profile revision is out of range");
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
 const auto optional_number = [&](const char* key, double& destination) {
  const core::Json* field = value.find(key);
  if (field == nullptr) return true;
  return number(value, key, destination, error);
 };
 if (!optional_number("valid_load_min_kg", parsed.valid_load_min_kg) ||
     !optional_number("valid_load_max_kg", parsed.valid_load_max_kg) ||
     !optional_number("created_unix_s", parsed.created_unix_s)) {
  return false;
 }
 if (!integer(value, "sample_count", integer_value, error)) return false;
 if (integer_value > std::numeric_limits<std::size_t>::max()) {
  fail(error, "calibration sample count is out of range");
  return false;
 }
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

bool validate_calibration_profile(const CalibrationArtifact& artifact,
                                  const CalibrationProfileRequirements& req,
                                  std::string* error) {
 if (!artifact.valid(error)) return false;
 if (req.robot_id.empty() || req.robot_description_hash == 0 ||
     req.profile_revision == 0) {
  fail(error, "calibration profile requirements are incomplete");
  return false;
 }
 if (artifact.robot_id != req.robot_id ||
     artifact.robot_description_hash != req.robot_description_hash ||
     artifact.profile_revision != req.profile_revision) {
  fail(error, "calibration artifact identity does not match the robot profile");
  return false;
 }
 if (nominal_digest(artifact.input_digest) && !req.allow_nominal) {
  fail(error, "nominal calibration requires explicit commissioning mode");
  return false;
 }
 if (req.temperature_c.has_value()) {
  const double temperature = *req.temperature_c;
  if (!finite(temperature) ||
      temperature < artifact.valid_temperature_min_c ||
      temperature > artifact.valid_temperature_max_c) {
   fail(error, "calibration artifact is outside its temperature range");
   return false;
  }
 }
 if (req.load_kg.has_value()) {
  const double load = *req.load_kg;
  if (!finite(load) ||
      (artifact.valid_load_min_kg == 0.0 &&
       artifact.valid_load_max_kg == 0.0) ||
      load < artifact.valid_load_min_kg ||
      load > artifact.valid_load_max_kg) {
   fail(error, "calibration artifact is outside its load range");
   return false;
  }
 }
 if (req.maximum_age_s.has_value()) {
  if (!req.now_unix_s.has_value() || !finite(*req.maximum_age_s) ||
      *req.maximum_age_s <= 0.0 || !finite(*req.now_unix_s) ||
      artifact.created_unix_s <= 0.0 ||
      *req.now_unix_s < artifact.created_unix_s ||
      *req.now_unix_s - artifact.created_unix_s > *req.maximum_age_s) {
   fail(error, "calibration artifact is stale or has no creation timestamp");
   return false;
  }
 }
 return true;
}

bool validate_native_hardware_profile(
    const NativeHardwareProfile& profile,
    const NativeHardwareProfileRequirements& req,
    std::string* error) {
 if (!iface::valid_chassis_capabilities(profile.chassis)) {
  fail(error, "native hardware chassis capability is invalid");
  return false;
 }
 if (req.profile_id.empty() || profile.chassis.profile_id != req.profile_id) {
  fail(error, "native hardware profile id does not match the board");
  return false;
 }
 if (profile.chassis.profile_revision != req.calibration.profile_revision) {
  fail(error, "native hardware profile revision does not match calibration");
  return false;
 }
 if (req.control_owner.has_value() &&
     profile.chassis.control_owner != *req.control_owner) {
  fail(error, "native hardware ownership does not match the profile");
  return false;
 }
 if (req.chassis_mode.has_value() &&
     profile.chassis.chassis_mode != *req.chassis_mode) {
  fail(error, "native hardware chassis mode does not match the profile");
  return false;
 }
 if (req.drive_count.has_value() &&
     profile.chassis.drive_count != *req.drive_count) {
  fail(error, "native hardware drive layout does not match the profile");
  return false;
 }
 if (req.steer_count.has_value() &&
     profile.chassis.steer_count != *req.steer_count) {
  fail(error, "native hardware steer layout does not match the profile");
  return false;
 }
 const bool nominal = nominal_digest(profile.calibration.input_digest);
 if (profile.source == NativeHardwareProfileSource::kNominalCommissioning) {
  if (!req.allow_nominal_commissioning) {
   fail(error, "nominal hardware profile requires explicit commissioning mode");
   return false;
  }
  // The source tag and artifact are one provenance contract. A caller may
  // opt into commissioning, but may not relabel a measured artifact as a
  // nominal one (or vice versa) to bypass the measured gate.
  if (!nominal || profile.calibration.sample_count != 0) {
   fail(error, "commissioning profile must contain a nominal artifact");
   return false;
  }
 } else if (profile.source == NativeHardwareProfileSource::kMeasured) {
  if (nominal) {
   fail(error, "nominal calibration cannot be presented as measured hardware");
   return false;
  }
 } else {
  fail(error, "native hardware profile source is invalid");
  return false;
 }
 CalibrationProfileRequirements calibration_requirements = req.calibration;
 calibration_requirements.allow_nominal = req.allow_nominal_commissioning;
 return validate_calibration_profile(profile.calibration,
                                     calibration_requirements, error);
}

bool make_nominal_native_hardware_profile(
    const RobotDescription& description,
    const iface::ChassisCapabilities& chassis,
    NativeHardwareProfile& out,
    std::string* error) {
 if (!valid_robot_description(description)) {
  fail(error, "cannot build nominal profile from invalid robot description");
  return false;
 }
 if (!iface::valid_chassis_capabilities(chassis)) {
  fail(error, "cannot build nominal profile from invalid chassis capability");
  return false;
 }
 if (chassis.profile_id.empty() || chassis.profile_revision == 0) {
  fail(error, "nominal profile requires a board profile id and revision");
  return false;
 }
 CalibrationArtifact artifact;
 artifact.robot_id = description.robot_id;
 artifact.robot_description_hash = hash_robot_description(description);
 artifact.profile_revision = chassis.profile_revision;
 artifact.input_digest =
     std::string(kNominalDigestPrefix) +
     std::to_string(artifact.robot_description_hash) + ":" +
     chassis.profile_id + ":" + std::to_string(chassis.profile_revision);
 // The declared robot geometry is the only available prior. It is explicit
 // and deterministic; no receive time, SIM state or hidden sensor estimate
 // manufactures a calibration result.
 artifact.wheel_radius_scale = 1.0;
 artifact.track_width_scale = 1.0;
 artifact.sample_count = 0;
 artifact.valid_temperature_min_c = -40.0;
 artifact.valid_temperature_max_c = 85.0;
 artifact.valid_load_min_kg = 0.0;
 artifact.valid_load_max_kg = 0.0;
 artifact.created_unix_s = 0.0;
 if (!artifact.valid(error)) return false;
 out = {};
 out.chassis = chassis;
 out.calibration = std::move(artifact);
 out.source = NativeHardwareProfileSource::kNominalCommissioning;
 return true;
}

bool save_calibration_artifact_atomic(const std::string& path,
                                      const CalibrationArtifact& artifact,
                                      std::string* error) {
 if (path.empty() || !artifact.valid(error)) return false;
 const std::filesystem::path destination(path);
 const std::filesystem::path temporary = destination.string() + ".tmp";
 {
  std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) {
   fail(error, "cannot open temporary calibration artifact");
   return false;
  }
  output << core::json_dump(artifact.to_json()) << '\n';
  output.flush();
  if (!output.good()) {
   output.close();
   std::error_code ignored;
   std::filesystem::remove(temporary, ignored);
   fail(error, "cannot write temporary calibration artifact");
   return false;
  }
 }
 std::error_code rename_error;
 std::filesystem::rename(temporary, destination, rename_error);
 if (rename_error) {
  std::error_code ignored;
  std::filesystem::remove(temporary, ignored);
  if (error != nullptr) {
   *error = "cannot atomically publish calibration artifact: " +
            rename_error.message();
  }
  return false;
 }
 return true;
}

bool load_calibration_artifact(const std::string& path,
                               CalibrationArtifact& out,
                               std::string* error) {
 if (path.empty()) {
  fail(error, "calibration artifact path is empty");
  return false;
 }
 std::ifstream input(path, std::ios::binary);
 if (!input.is_open()) {
  fail(error, "cannot open calibration artifact");
  return false;
 }
 std::ostringstream text;
 text << input.rdbuf();
 if (!input.good() && !input.eof()) {
  fail(error, "cannot read calibration artifact");
  return false;
 }
 std::string parse_error;
 const auto parsed = core::json_parse(text.str(), &parse_error);
 if (parsed == nullptr || !calibration_from_json(*parsed, out, error)) {
  if (error != nullptr && error->empty()) *error = parse_error;
  return false;
 }
 return true;
}

bool load_calibration_profile(const std::string& path,
                             const CalibrationProfileRequirements& req,
                             CalibrationArtifact& out,
                             std::string* error) {
 CalibrationArtifact loaded;
 if (!load_calibration_artifact(path, loaded, error) ||
     !validate_calibration_profile(loaded, req, error)) {
  return false;
 }
 out = std::move(loaded);
 return true;
}

bool CalibrationJournalEntry::valid(std::string* error) const {
 if (sequence == 0 || !finite(timestamp_s) || timestamp_s < 0.0 ||
     artifact_digest.empty() || event.empty() || !finite(temperature_c) ||
     !finite(load_kg) || load_kg < 0.0 || !finite(wheel_radius_scale) ||
     !finite(residual) || residual < 0.0) {
  fail(error, "calibration journal entry is invalid");
  return false;
 }
 return true;
}

core::Json CalibrationJournalEntry::to_json() const {
 return core::Json::Obj({
     {"sequence", core::Json::Num(static_cast<double>(sequence))},
     {"timestamp_s", core::Json::Num(timestamp_s)},
     {"artifact_digest", core::Json::Str(artifact_digest)},
     {"event", core::Json::Str(event)},
     {"temperature_c", core::Json::Num(temperature_c)},
     {"load_kg", core::Json::Num(load_kg)},
     {"wheel_radius_scale", core::Json::Num(wheel_radius_scale)},
     {"residual", core::Json::Num(residual)},
     {"accepted", core::Json::Bool(accepted)},
     {"diagnostic", core::Json::Str(diagnostic)}});
}

namespace {

bool calibration_journal_from_json(const core::Json& value,
                                   CalibrationJournalEntry& out,
                                   std::string* error) {
 if (!value.is_object()) {
  fail(error, "calibration journal entry must be an object");
  return false;
 }
 CalibrationJournalEntry parsed;
 std::uint64_t sequence = 0;
 if (!integer(value, "sequence", sequence, error) ||
     !number(value, "timestamp_s", parsed.timestamp_s, error) ||
     !string(value, "artifact_digest", parsed.artifact_digest, error) ||
     !string(value, "event", parsed.event, error) ||
     !number(value, "temperature_c", parsed.temperature_c, error) ||
     !number(value, "load_kg", parsed.load_kg, error) ||
     !number(value, "wheel_radius_scale", parsed.wheel_radius_scale, error) ||
     !number(value, "residual", parsed.residual, error)) {
  return false;
 }
 const core::Json* accepted = value.find("accepted");
 if (accepted == nullptr || !accepted->is_bool()) {
  fail(error, "calibration journal accepted field is invalid");
  return false;
 }
 parsed.sequence = sequence;
 parsed.accepted = accepted->as_bool();
 const core::Json* diagnostic = value.find("diagnostic");
 if (diagnostic != nullptr) {
  if (!diagnostic->is_str()) {
   fail(error, "calibration journal diagnostic field is invalid");
   return false;
  }
  parsed.diagnostic = diagnostic->as_str();
 }
 if (!parsed.valid(error)) return false;
 out = std::move(parsed);
 return true;
}

} // namespace

bool append_calibration_journal(const std::string& path,
                                const CalibrationJournalEntry& entry,
                                std::string* error) {
 if (path.empty() || !entry.valid(error)) return false;
 std::ofstream output(path, std::ios::out | std::ios::app);
 if (!output.is_open()) {
  fail(error, "cannot open calibration journal");
  return false;
 }
 output << core::json_dump(entry.to_json()) << '\n';
 output.flush();
 if (!output.good()) {
  fail(error, "cannot append calibration journal");
  return false;
 }
 return true;
}

bool load_calibration_journal(const std::string& path,
                              std::vector<CalibrationJournalEntry>& out,
                              std::string* error) {
 if (path.empty()) {
  fail(error, "calibration journal path is empty");
  return false;
 }
 std::ifstream input(path);
 if (!input.is_open()) {
  fail(error, "cannot open calibration journal");
  return false;
 }
 std::vector<CalibrationJournalEntry> parsed;
 std::string line;
 std::size_t line_number = 0;
 while (std::getline(input, line)) {
  ++line_number;
  const auto first = line.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) continue;
  std::string parse_error;
  const auto value = core::json_parse(line.substr(first), &parse_error);
  CalibrationJournalEntry entry;
  if (value == nullptr ||
      !calibration_journal_from_json(*value, entry, &parse_error)) {
   if (error != nullptr) {
    *error = "calibration journal line " + std::to_string(line_number) +
             " is invalid: " + parse_error;
   }
   return false;
  }
  if (!parsed.empty() && entry.sequence <= parsed.back().sequence) {
   if (error != nullptr) {
    *error = "calibration journal sequence is not strictly increasing";
   }
   return false;
  }
  parsed.push_back(std::move(entry));
 }
 if (!input.eof() && input.fail()) {
  fail(error, "cannot read calibration journal");
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
    const CalibrationMeasurement* previous =
        parsed.empty() ? nullptr : &parsed.back();
    if (!valid_measurement_sample(sample, previous)) {
      if (error != nullptr) {
        *error = "calibration CSV line " + std::to_string(line_number) +
                 " is not finite and strictly time ordered";
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
