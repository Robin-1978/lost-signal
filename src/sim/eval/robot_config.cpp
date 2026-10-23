#include "sim/eval/robot_config.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "sim/eval/json.hpp"
#include "sim/world/motor.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::eval {

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

bool take_str(const Json& obj, const char* key, std::string& out,
              bool& present, std::string& err) {
  const Json* node = obj.find(key);
  present = (node != nullptr);
  if (!present) return true;
  if (!node->is_str()) {
    err = std::string("key '") + key + "' not a string";
    return false;
  }
  out = node->as_str();
  return true;
}

// Same plant keys + rules as the drive-profile loader (one contract).
bool merge_drive_keys(const Json& obj, robot::platform::DriveProfile& merged,
                     std::string& err) {
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
  return true;
}

bool load_vec3(const Json& obj, const char* key, double out[3],
               std::string& err) {
  const Json* node = obj.find(key);
  if (node == nullptr || !node->is_array() ||
      node->as_array().size() != 3) {
    err = std::string("key '") + key + "' must be a 3-number array";
    return false;
  }
  for (int i = 0; i < 3; ++i) {
    if (!node->as_array()[static_cast<std::size_t>(i)].is_num() ||
        !std::isfinite(
            node->as_array()[static_cast<std::size_t>(i)].as_num())) {
      err = std::string("key '") + key + "' must be finite numbers";
      return false;
    }
    out[i] = node->as_array()[static_cast<std::size_t>(i)].as_num();
  }
  return true;
}

}  // namespace

bool load_robot_config(const std::string& path, RobotConfig& config,
                       std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = json_parse(text, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid robot config: " + parse_error;
    return false;
  }
  const Json& obj = *root;
  RobotConfig out;
  // Kind: "kind" (drive-profile compat), "chassis_kind", or both agreeing.
  std::string kind, kind2;
  bool p1 = false, p2 = false;
  if (!take_str(obj, "kind", kind, p1, err)) return false;
  if (!take_str(obj, "chassis_kind", kind2, p2, err)) return false;
  if (!p1 && !p2) {
    err = "robot config needs a string 'kind' (or 'chassis_kind')";
    return false;
  }
  if (p1 && p2 && kind != kind2) {
    err = "'kind' and 'chassis_kind' disagree";
    return false;
  }
  const std::string& want = p1 ? kind : kind2;
  if (!robot::platform::DriveProfile::for_chassis(want, out.profile)) {
    err = "unknown chassis kind: " + want;
    return false;
  }
  if (!merge_drive_keys(obj, out.profile, err)) return false;
  // Nested "drive" object overrides flat keys (seeds use it for
  // readability; bare drive-profile JSON stays loadable as a config).
  const Json* drive = obj.find("drive");
  if (drive != nullptr) {
    if (!drive->is_object()) {
      err = "'drive' must be an object";
      return false;
    }
    if (!merge_drive_keys(*drive, out.profile, err)) return false;
  }
  // Limits (all optional; finite non-negative).
  const Json* lim = obj.find("limits");
  if (lim != nullptr) {
    if (!lim->is_object()) {
      err = "'limits' must be an object";
      return false;
    }
    double v = 0.0;
    bool present = false;
    struct Slot {
      const char* key;
      double* slot;
    };
    Slot slots[] = {
        {"max_forward_mps", &out.limits.max_forward_mps},
        {"max_yaw_rate_radps", &out.limits.max_yaw_rate_radps},
        {"footprint_radius_m", &out.limits.footprint_radius_m},
        {"max_steer_rad", &out.limits.max_steer_rad},
        {"max_lateral_mps", &out.limits.max_lateral_mps},
    };
    for (const auto& s : slots) {
      if (!take_num(*lim, s.key, v, present, err)) return false;
      if (present) {
        if (!std::isfinite(v) || v < 0.0) {
          err = std::string("limits.'") + s.key +
                "' must be finite non-negative";
          return false;
        }
        *s.slot = v;
      }
    }
  }
  // Sensors + links.
  const Json* sens = obj.find("sensors");
  if (sens != nullptr) {
    if (!sens->is_array()) {
      err = "'sensors' must be an array";
      return false;
    }
    for (const auto& e : sens->as_array()) {
      if (!e.is_object()) {
        err = "sensor entry must be an object";
        return false;
      }
      robot::platform::SensorMount m;
      std::string id, link;
      bool pi = false, pl = false;
      if (!take_str(e, "sensor_id", id, pi, err) || !pi || id.empty()) {
        err = "sensor needs a non-empty 'sensor_id'";
        return false;
      }
      if (!take_str(e, "link", link, pl, err) || !pl || link.empty()) {
        err = "sensor needs a non-empty 'link'";
        return false;
      }
      m.sensor_id = id;
      m.link = link;
      if (!load_vec3(e, "xyz_m", m.xyz_m, err)) return false;
      if (!load_vec3(e, "rpy_rad", m.rpy_rad, err)) return false;
      out.sensors.push_back(m);
    }
  }
  const Json* links = obj.find("links");
  if (links != nullptr) {
    if (!links->is_array()) {
      err = "'links' must be an array";
      return false;
    }
    for (const auto& e : links->as_array()) {
      if (!e.is_object()) {
        err = "link entry must be an object";
        return false;
      }
      robot::platform::LinkMount m;
      std::string link, parent;
      bool pl = false, pp = false;
      if (!take_str(e, "link", link, pl, err) || !pl || link.empty()) {
        err = "link entry needs a non-empty 'link'";
        return false;
      }
      if (!take_str(e, "parent", parent, pp, err) || !pp ||
          parent.empty()) {
        err = "link entry needs a non-empty 'parent'";
        return false;
      }
      m.link = link;
      m.parent = parent;
      if (!load_vec3(e, "xyz_m", m.xyz_m, err)) return false;
      if (!load_vec3(e, "rpy_rad", m.rpy_rad, err)) return false;
      out.links.push_back(m);
    }
  }
  // Brake + identity.
  const Json* brake = obj.find("brake");
  if (brake != nullptr) {
    if (!brake->is_object()) {
      err = "'brake' must be an object";
      return false;
    }
    double v = 0.0;
    bool present = false;
    struct Slot {
      const char* key;
      double* slot;
    };
    Slot slots[] = {
        {"max_torque_Nm", &out.brake_max_torque_Nm},
        {"apply_time_s", &out.brake_apply_time_s},
        {"release_time_s", &out.brake_release_time_s},
    };
    for (const auto& s : slots) {
      if (!take_num(*brake, s.key, v, present, err)) return false;
      if (present) {
        if (!std::isfinite(v) || v < 0.0) {
          err = std::string("brake.'") + s.key +
                "' must be finite non-negative";
          return false;
        }
        *s.slot = v;
      }
    }
  }
  std::string rid;
  bool pr = false;
  if (!take_str(obj, "robot_id", rid, pr, err)) return false;
  if (pr) {
    if (rid.empty()) {
      err = "'robot_id' must be non-empty";
      return false;
    }
    out.robot_id = rid;
  }
  const Json* ver = obj.find("version");
  if (ver != nullptr) {
    if (!ver->is_num() || ver->as_num() <= 0.0 ||
        ver->as_num() != std::floor(ver->as_num())) {
      err = "'version' must be a positive integer";
      return false;
    }
    out.version = static_cast<std::uint64_t>(ver->as_num());
  }
  config = out;
  err.clear();
  return true;
}

robot::platform::RobotDescription description_from_config(
    const RobotConfig& config) {
  robot::platform::RobotDescription desc;
  desc.robot_id = config.robot_id;
  desc.version = config.version;
  desc.links = config.links;
  desc.sensors = config.sensors;
  desc.limits = config.limits;
  desc.chassis_kind = config.profile.chassis_kind;
  return desc;
}

world::MotorParams motor_from_config(const RobotConfig& config) {
  world::MotorParams motor = world::motor_params_from_profile(config.profile);
  motor.brake_max_torque_Nm = config.brake_max_torque_Nm;
  motor.brake_apply_time_s = config.brake_apply_time_s;
  motor.brake_release_time_s = config.brake_release_time_s;
  return motor;
}

}  // namespace lost_signal::sim::eval
