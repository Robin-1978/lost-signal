#include "sim/eval/sim_config.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::sim::eval {
using robot::core::Json;
using robot::core::json_dump;
using robot::core::json_parse;

namespace {

bool read_text(const std::string& path, std::string& text, std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    if (error) *error = "cannot open file: " + path;
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  text = ss.str();
  return true;
}

bool take_num(const Json& obj, const char* key, double& out, bool& present,
              std::string& err) {
  const Json* node = obj.find(key);
  present = (node != nullptr);
  if (!present) return true;
  if (!node->is_num()) {
    err = std::string("key '") + key + "' not a number";
    return false;
  }
  out = node->as_num();
  return true;
}

bool finite_non_negative(double v, const char* what, std::string& err) {
  if (!std::isfinite(v) || v < 0.0) {
    err = std::string(what) + " must be finite non-negative";
    return false;
  }
  return true;
}

bool on_physics_grid(double seconds) {
 if (seconds == 0.0) return true;
 constexpr double kPhysicsTickS = 0.001;
 const double ticks = seconds / kPhysicsTickS;
 return std::abs(ticks - std::round(ticks)) <= 1e-9;
}

}  // namespace

bool resolve_sensor_realism_profile(
    const SensorRealismProfileConfig& profile,
    providers::DeliveryConfig& delivery,
    providers::ImuRealismConfig& imu,
    providers::LidarDeviceModel& lidar,
    std::string* error) {
  std::string local_error;
  std::string& err = error != nullptr ? *error : local_error;
  if (profile.version != 1) {
    err = "sensor_realism.version must be 1";
    return false;
  }
  if (profile.name != "ideal" && profile.name != "nominal" &&
      profile.name != "adverse") {
    err = "sensor_realism.profile must be one of ideal, nominal, adverse";
    return false;
  }
  // Zero is a valid deterministic seed; profile names select behavior and no
  // hidden random seed is introduced.
  providers::DeliveryConfig next_delivery;
  providers::ImuRealismConfig next_imu;
  providers::LidarDeviceModel next_lidar;
  next_delivery.seed = profile.seed;
  next_imu.seed = profile.seed;
  if (profile.name == "nominal") {
    // Conservative device-level imperfections suitable for the published
    // default.  They are centralized profile data, not scene thresholds.
    next_imu.gyro_noise_radps = 0.0015;
    next_imu.gyro_bias_stddev_radps = 0.004;
    next_imu.gyro_bias_walk_radps_sqrt_s = 0.0001;
    next_imu.accel_noise_mps2 = 0.02;
    next_lidar.range_stddev_m = 0.003;
    next_lidar.range_resolution_m = 0.002;
    next_lidar.dropout_probability = 0.002;
    next_lidar.min_range_m = 0.05;
  } else if (profile.name == "adverse") {
    next_delivery.latency_steps = 2;
    next_delivery.drop_probability = 0.08;
    next_imu.gyro_noise_radps = 0.01;
    next_imu.gyro_bias_stddev_radps = 0.03;
    next_imu.gyro_bias_walk_radps_sqrt_s = 0.001;
    next_imu.accel_noise_mps2 = 0.12;
    next_lidar.range_stddev_m = 0.02;
    next_lidar.range_bias_m = 0.01;
    next_lidar.range_resolution_m = 0.01;
    next_lidar.dropout_probability = 0.12;
    next_lidar.min_range_m = 0.08;
  }
  if (!providers::valid_device_model(next_lidar)) {
    err = "sensor_realism profile resolved an invalid lidar model";
    return false;
  }
  delivery = next_delivery;
  imu = next_imu;
  lidar = next_lidar;
  err.clear();
  return true;
}

bool load_sim_config(const std::string& path, SimConfig& config,
                     std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = json_parse(text, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid sim config: " + parse_error;
    return false;
  }
  const Json& obj = *root;
  SimConfig out;
  bool profile_present = false;
  const Json* profile = obj.find("sensor_realism");
  if (profile != nullptr) {
    profile_present = true;
    if (!profile->is_object()) {
      err = "'sensor_realism' must be an object";
      return false;
    }
    const Json* version = profile->find("version");
    const Json* name = profile->find("profile");
    if (version == nullptr || !version->is_num() ||
        version->as_num() != 1.0 || name == nullptr || !name->is_str()) {
      err = "sensor_realism requires version 1 and a profile name";
      return false;
    }
    out.sensor_realism.version = 1;
    out.sensor_realism.name = name->as_str();
    const Json* seed = profile->find("seed");
    if (seed != nullptr) {
      if (!seed->is_num() || seed->as_num() < 0.0 ||
          seed->as_num() != std::floor(seed->as_num()) ||
          seed->as_num() > static_cast<double>(
              std::numeric_limits<std::uint64_t>::max())) {
        err = "sensor_realism.'seed' must be a non-negative integer";
        return false;
      }
      out.sensor_realism.seed = static_cast<std::uint64_t>(seed->as_num());
    }
    if (!resolve_sensor_realism_profile(out.sensor_realism, out.delivery,
                                        out.imu_realism, out.lidar_realism,
                                        &err)) {
      return false;
    }
  }
  // Cadence (0 disables a tap, matching set_cadence fail-closed rules).
  const Json* cad = obj.find("cadence");
  if (cad != nullptr) {
    if (!cad->is_object()) {
      err = "'cadence' must be an object";
      return false;
    }
    double v = 0.0;
    bool present = false;
    struct Slot {
      const char* key;
      double* slot;
    };
 Slot slots[] = {{"scan_s", &out.scan_period_s},
 {"motion_s", &out.motion_period_s},
 {"imu_s", &out.imu_period_s},
 {"decision_s", &out.decision_period_s},
 {"planning_s", &out.planning_period_s},
                    {"record_s", &out.record_period_s}};
    for (const auto& s : slots) {
      if (!take_num(*cad, s.key, v, present, err)) return false;
      if (present) {
        if (!finite_non_negative(v, s.key, err)) return false;
 if (!on_physics_grid(v)) {
 err = std::string("cadence '") + s.key +
 "' must be an integer multiple of the 1 ms physics tick";
 return false;
 }
        *s.slot = v;
      }
    }
  }
  double v = 0.0;
  bool present = false;
  if (!take_num(obj, "judge_radius_m", v, present, err)) return false;
  if (present) {
    if (!finite_non_negative(v, "'judge_radius_m'", err)) return false;
    out.judge_radius_m = v;
  }
 // SIM-only objective acceptance. This never changes ROBOT arrival policy;
 // it independently verifies the physical pose and motion at completion.
 const Json* acceptance = obj.find("objective_acceptance");
 if (acceptance != nullptr) {
 if (!acceptance->is_object()) {
 err = "'objective_acceptance' must be an object";
 return false;
 }
 const Json* version = acceptance->find("version");
 if (version == nullptr || !version->is_num() ||
 version->as_num() != 2.0) {
 err = "objective_acceptance.'version' must be 2";
 return false;
 }
 struct AcceptanceSlot {
 const char* key;
 double* slot;
 };
 AcceptanceSlot slots[] = {
 {"position_tolerance_m",
 &out.objective_acceptance.position_tolerance_m},
 {"heading_tolerance_rad",
 &out.objective_acceptance.heading_tolerance_rad},
 {"maximum_linear_speed_mps",
 &out.objective_acceptance.maximum_linear_speed_mps},
 {"maximum_yaw_rate_radps",
 &out.objective_acceptance.maximum_yaw_rate_radps},
 {"terminal_observation_s",
 &out.objective_acceptance.terminal_observation_s}};
 for (const auto& slot : slots) {
 if (!take_num(*acceptance, slot.key, v, present, err) || !present ||
 !std::isfinite(v) || !(v > 0.0)) {
 err = std::string("objective_acceptance.'") + slot.key +
 "' must be finite positive";
 return false;
 }
 *slot.slot = v;
 }
 const Json* stable = acceptance->find("stable_steps");
 if (stable == nullptr || !stable->is_num() ||
 !std::isfinite(stable->as_num()) || stable->as_num() < 1.0 ||
 stable->as_num() != std::floor(stable->as_num()) ||
 stable->as_num() >
 static_cast<double>(std::numeric_limits<std::uint32_t>::max())) {
 err = "objective_acceptance.'stable_steps' must be a positive integer";
 return false;
 }
 out.objective_acceptance.stable_steps =
 static_cast<std::uint32_t>(stable->as_num());
 }
  // Delivery shaping.
  const Json* del = obj.find("delivery");
  if (del != nullptr) {
    if (profile_present) {
      err = "sensor_realism cannot be combined with legacy 'delivery'";
      return false;
    }
    if (!del->is_object()) {
      err = "'delivery' must be an object";
      return false;
    }
    const Json* seed = del->find("seed");
    if (seed != nullptr) {
      if (!seed->is_num() || seed->as_num() < 0.0 ||
          seed->as_num() != std::floor(seed->as_num())) {
        err = "delivery.'seed' must be a non-negative integer";
        return false;
      }
      out.delivery.seed =
          static_cast<std::uint64_t>(seed->as_num());
    }
    const Json* lat = del->find("latency_steps");
    if (lat != nullptr) {
      if (!lat->is_num() || lat->as_num() < 0.0 ||
          lat->as_num() != std::floor(lat->as_num())) {
        err = "delivery.'latency_steps' must be a non-negative integer";
        return false;
      }
      out.delivery.latency_steps = static_cast<int>(lat->as_num());
    }
    const Json* drop = del->find("drop_probability");
    if (drop != nullptr) {
      if (!drop->is_num() || drop->as_num() < 0.0 || drop->as_num() > 1.0) {
        err = "delivery.'drop_probability' must be in [0, 1]";
        return false;
      }
      out.delivery.drop_probability = drop->as_num();
    }
  }
  // IMU realism.
  const Json* imu = obj.find("imu_realism");
  if (imu != nullptr) {
    if (profile_present) {
      err = "sensor_realism cannot be combined with legacy 'imu_realism'";
      return false;
    }
    if (!imu->is_object()) {
      err = "'imu_realism' must be an object";
      return false;
    }
    const Json* seed = imu->find("seed");
    if (seed != nullptr) {
      if (!seed->is_num() || seed->as_num() < 0.0 ||
          seed->as_num() != std::floor(seed->as_num())) {
        err = "imu_realism.'seed' must be a non-negative integer";
        return false;
      }
      out.imu_realism.seed = static_cast<std::uint64_t>(seed->as_num());
    }
    struct Slot {
      const char* key;
      double* slot;
    };
    Slot slots[] = {
        {"gyro_noise_radps", &out.imu_realism.gyro_noise_radps},
        {"gyro_trim_z_radps", &out.imu_realism.gyro_trim_radps[2]},
        {"gyro_bias_stddev_radps",
         &out.imu_realism.gyro_bias_stddev_radps},
        {"gyro_bias_walk_radps_sqrt_s",
         &out.imu_realism.gyro_bias_walk_radps_sqrt_s},
        {"accel_noise_mps2", &out.imu_realism.accel_noise_mps2},
    };
    for (const auto& s : slots) {
      if (!take_num(*imu, s.key, v, present, err)) return false;
      if (present) {
        if (!std::isfinite(v) || v < 0.0) {
          err = std::string("imu_realism.'") + s.key +
                "' must be finite non-negative";
          return false;
        }
        *s.slot = v;
      }
    }
  }
  // Lidar geometry (provider construction parameters, SIM harness only).
  const Json* lidar = obj.find("lidar");
  if (lidar != nullptr) {
    if (!lidar->is_object()) {
      err = "'lidar' must be an object";
      return false;
    }
    const Json* beams = lidar->find("beams");
    if (beams != nullptr) {
      if (!beams->is_num() || beams->as_num() <= 0.0 ||
          beams->as_num() != std::floor(beams->as_num())) {
        err = "lidar.'beams' must be a positive integer";
        return false;
      }
      out.lidar_beams = static_cast<int>(beams->as_num());
    }
    if (!take_num(*lidar, "max_range_m", v, present, err)) return false;
    if (present) {
      if (!std::isfinite(v) || v <= 0.0) {
        err = "lidar.'max_range_m' must be finite positive";
        return false;
      }
      out.lidar_max_range_m = static_cast<float>(v);
    }
    const Json* rings = lidar->find("rings");
    if (rings != nullptr) {
      if (!rings->is_num() || rings->as_num() <= 0.0 ||
          rings->as_num() != std::floor(rings->as_num())) {
        err = "lidar.'rings' must be a positive integer";
        return false;
      }
      out.lidar_rings = static_cast<int>(rings->as_num());
    }
    const Json* composite = lidar->find("composite");
    if (composite != nullptr) {
      if (!composite->is_bool()) {
        err = "lidar.'composite' must be a boolean";
        return false;
      }
      out.lidar_composite = composite->as_bool();
    }
  }
  config = out;
  err.clear();
  return true;
}

std::optional<SimConfig> load_default_sim_config(
    const std::string& explicit_path, std::string* used_path,
    std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  SimConfig out;
  if (!explicit_path.empty()) {
    std::string load_err;
    if (!load_sim_config(explicit_path, out, &load_err)) {
      err = load_err;
      return std::nullopt;
    }
    if (used_path != nullptr) *used_path = explicit_path;
    err.clear();
    return out;
  }
  constexpr char kSeed[] = "configs/sim/default.json";
  std::ifstream probe(kSeed, std::ios::binary);
  if (probe.is_open()) {
    probe.close();
    std::string load_err;
    if (!load_sim_config(kSeed, out, &load_err)) {
      err = load_err;
      return std::nullopt;
    }
    if (used_path != nullptr) *used_path = kSeed;
    err.clear();
    return out;
  }
  if (used_path != nullptr) *used_path = "(code defaults)";
  err.clear();
  return std::nullopt;
}

}  // namespace lost_signal::sim::eval
