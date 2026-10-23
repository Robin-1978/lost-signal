#include "sim/eval/drive_profile_io.hpp"

#include <cmath>
#include <fstream>
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

}  // namespace

bool load_drive_profile(const std::string& path,
                        robot::platform::DriveProfile& profile,
                        std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = json_parse(text, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid drive profile: " + parse_error;
    return false;
  }
  const Json& obj = *root;
  const Json* kind = obj.find("kind");
  if (kind == nullptr || !kind->is_str()) {
    err = "drive profile needs a string 'kind'";
    return false;
  }
  robot::platform::DriveProfile merged;
  if (!robot::platform::DriveProfile::for_chassis(kind->as_str(), merged)) {
    err = "unknown chassis kind: " + kind->as_str();
    return false;
  }
  double v = 0.0;
  bool present = false;
  const char* keys[] = {"wheel_radius_m",
                        "wheel_base_m",
                        "track_width_m",
                        "free_speed_mps",
                        "stall_force_N",
                        "wheel_equiv_mass_kg",
                        "vehicle_mass_kg",
                        "rolling_resist",
                        "viscous_drag_N_per_mps",
                        "coulomb_smoothing_mps",
                        "steer_stall_torque_Nm",
                        "tire_friction_mu",
                        "max_steer_rad",
                        "max_steer_rate_radps",
                        "steer_time_constant_s"};
  double* slots[] = {&merged.wheel_radius_m,
                     &merged.wheel_base_m,
                     &merged.track_width_m,
                     &merged.free_speed_mps,
                     &merged.stall_force_N,
                     &merged.wheel_equiv_mass_kg,
                     &merged.vehicle_mass_kg,
                     &merged.rolling_resist,
                     &merged.viscous_drag_N_per_mps,
                     &merged.coulomb_smoothing_mps,
                     &merged.steer_stall_torque_Nm,
                     &merged.tire_friction_mu,
                     &merged.max_steer_rad,
                     &merged.max_steer_rate_radps,
                     &merged.steer_time_constant_s};
  for (int i = 0; i < 15; ++i) {
    if (!take_num(obj, keys[i], v, present, err)) return false;
    if (present) {
      if (!std::isfinite(v) || v <= 0.0) {
        err = std::string("key '") + keys[i] + "' must be finite positive";
        return false;
      }
      *slots[i] = v;
    }
  }
  const Json* enc = obj.find("encoder_counts_per_revolution");
  if (enc != nullptr) {
    if (!enc->is_num() || enc->as_num() <= 0.0 ||
        enc->as_num() != std::floor(enc->as_num())) {
      err = "key 'encoder_counts_per_revolution' must be a positive integer";
      return false;
    }
    merged.encoder_counts_per_revolution = static_cast<int>(enc->as_num());
  }
  // Yaw scrub (tracked): zero is valid, so outside the positive-only loop.
  const char* scrub_keys[] = {"yaw_scrub_coulomb_Nm",
                              "yaw_scrub_viscous_Nm_per_radps"};
  double* scrub_slots[] = {&merged.yaw_scrub_coulomb_Nm,
                           &merged.yaw_scrub_viscous_Nm_per_radps};
  for (int i = 0; i < 2; ++i) {
    if (!take_num(obj, scrub_keys[i], v, present, err)) return false;
    if (present) {
      if (!std::isfinite(v) || v < 0.0) {
        err = std::string("key '") + scrub_keys[i] +
              "' must be finite non-negative";
        return false;
      }
      *scrub_slots[i] = v;
    }
  }
  profile = merged;
  return true;
}

}  // namespace lost_signal::sim::eval
