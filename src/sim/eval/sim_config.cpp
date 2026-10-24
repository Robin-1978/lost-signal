#include "sim/eval/sim_config.hpp"

#include <cmath>
#include <fstream>
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

}  // namespace

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
                    {"record_s", &out.record_period_s}};
    for (const auto& s : slots) {
      if (!take_num(*cad, s.key, v, present, err)) return false;
      if (present) {
        if (!finite_non_negative(v, s.key, err)) return false;
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
  // Delivery shaping.
  const Json* del = obj.find("delivery");
  if (del != nullptr) {
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
