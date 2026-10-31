#include "robot/platform/robot_config.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::robot::platform {

using core::Json;

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

bool merge_drive_profile_keys(const Json& obj, DriveProfile& merged,
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
  // Yaw scrub (tracked): zero is valid (wheeled chassis pivot freely), so
  // these live outside the positive-only loop above. Finite, >= 0.
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
  return true;
}

// Chassis hard geometry ("chassis" object): body/deck size, drive axle
// offset, tire width, unpowered supports. Fail closed on any malformed
// entry (a typo'd support must refuse, never silently drop a contact).
// Pure range validation, shared by the merge path and the absent-block
// path (defaults still checked against the merged wheel radius).
bool validate_chassis_geometry(const DriveProfile& merged, std::string& err) {
  // Effective COM (0 = follow the axle — resolved here so every caller
  // sees one number, never two rules). Negative is refused (typo guard;
  // auto is spelled 0, exactly).
  if (!(merged.com_height_m >= 0.0)) {
    err = "chassis.'com_height_m' must be >= 0 (0 = follow the axle)";
    return false;
  }
  const double com_h =
      merged.com_height_m > 0.0 ? merged.com_height_m : merged.wheel_radius_m;
  if (!(merged.body_half_height_m > 0.0) ||
      !(merged.body_half_height_m < merged.wheel_radius_m)) {
    err = "chassis.'body_half_height_m' must be in (0, wheel_radius_m)";
    return false;
  }
 if (!(com_h > 0.0) || !std::isfinite(com_h)) {
 err = "chassis COM height must be finite positive";
    return false;
  }
 if (!(merged.body_half_length_m > 0.0) ||
 !(merged.body_half_width_m > 0.0)) {
 err = "chassis body half-length and half-width must be positive";
    return false;
  }
  if (!(merged.tire_half_width_m > 0.0)) {
    err = "chassis.'tire_half_width_m' must be positive";
    return false;
  }
  if (!(merged.wheel_mass_kg > 0.0)) {
    err = "chassis.'wheel_mass_kg' must be positive";
    return false;
  }
  if (!(merged.knuckle_mass_kg > 0.0)) {
    err = "chassis.'knuckle_mass_kg' must be positive";
    return false;
  }
  if (!(merged.payload_mass_kg >= 0.0) ||
      !std::isfinite(merged.payload_mass_kg)) {
    err = "chassis.'payload_mass_kg' must be finite non-negative";
    return false;
  }
  if (!(merged.payload_height_m >= 0.0) ||
      !std::isfinite(merged.payload_height_m)) {
    err = "chassis.'payload_height_m' must be finite non-negative";
    return false;
  }
  if (!(merged.idler_trail_m >= 0.0) ||
      !std::isfinite(merged.idler_trail_m)) {
    err = "chassis.'idler_trail_m' must be finite non-negative";
    return false;
  }
  return true;
}

bool merge_chassis_keys(const Json& obj, DriveProfile& merged,
                        std::string& err) {
  double v = 0.0;
  bool present = false;
 const char* keys[] = {"body_half_height_m", "body_half_length_m",
 "body_half_width_m",
                        "drive_axle_x_m", "tire_half_width_m",
                        "wheel_mass_kg", "knuckle_mass_kg",
 "com_height_m",
                        "payload_mass_kg", "payload_height_m",
                        "idler_trail_m"};
 double* slots[] = {&merged.body_half_height_m, &merged.body_half_length_m,
 &merged.body_half_width_m,
                     &merged.drive_axle_x_m, &merged.tire_half_width_m,
                     &merged.wheel_mass_kg, &merged.knuckle_mass_kg,
 &merged.com_height_m,
                     &merged.payload_mass_kg, &merged.payload_height_m,
                     &merged.idler_trail_m};
  for (int i = 0; i < 11; ++i) {
    if (!take_num(obj, keys[i], v, present, err)) return false;
    if (present) {
      if (!std::isfinite(v)) {
        err = std::string("chassis.'") + keys[i] + "' must be finite";
        return false;
      }
      *slots[i] = v;
    }
  }
  // Range doctrine (checked again engine-side at build; the loader rejects
  // nonsense early so bad authoring fails at parse, not at first step):
  // deck must clear the floor (belly = com - half_h > 0), tub positive,
  // tire width positive. Axle offset is bounded at build against base_len.
  if (!validate_chassis_geometry(merged, err)) return false;
  const Json* sup = obj.find("supports");
  if (sup == nullptr) return true;  // absent = default caster pair at convert
  if (!sup->is_array()) {
    err = "'chassis.supports' must be an array";
    return false;
  }
  std::vector<DriveProfile::SupportSpec> parsed;
  for (const auto& e : sup->as_array()) {
    if (!e.is_object()) {
      err = "support entry must be an object";
      return false;
    }
    DriveProfile::SupportSpec s;
    std::string kind;
    bool has_kind = false;
    if (!take_str(e, "kind", kind, has_kind, err)) return false;
    if (has_kind) s.kind = kind;
    if (s.kind != "caster") {
      err = std::string("support kind '") + s.kind +
 "' unsupported (only 'caster')";
      return false;
    }
    const char* skeys[] = {"x_m", "y_m", "trail_m", "radius_m", "mass_kg",
 "friction_mu", "swivel_damping_Nms", "swivel_friction_Nm",
      "rolling_friction_m"};
    double* sslots[] = {&s.x_m, &s.y_m, &s.trail_m, &s.radius_m, &s.mass_kg,
 &s.friction_mu, &s.swivel_damping_Nms, &s.swivel_friction_Nm,
      &s.rolling_friction_m};
    for (int i = 0; i < 9; ++i) {
      if (!take_num(e, skeys[i], v, present, err)) return false;
      if (present) {
        if (!std::isfinite(v)) {
          err = std::string("support.'") + skeys[i] + "' must be finite";
          return false;
        }
        *sslots[i] = v;
      }
    }
    if (!(s.trail_m >= 0.0) || !(s.radius_m > 0.0) || !(s.mass_kg > 0.0) ||
        !(s.friction_mu >= 0.0) || !(s.swivel_damping_Nms >= 0.0) ||
        !(s.swivel_friction_Nm >= 0.0) || !(s.rolling_friction_m >= 0.0)) {
      err = "caster needs trail/frictions >= 0 and radius_m/mass_kg > 0";
      return false;
    }
    parsed.push_back(s);
  }
  if (parsed.size() > 4) {
    err = "'chassis.supports' holds at most 4 entries";
    return false;
  }
  merged.supports = std::move(parsed);
  return true;
}

bool merge_chassis_config(const Json* chassis_obj, DriveProfile& merged,
                          std::string& err) {
  if (chassis_obj == nullptr) return validate_chassis_geometry(merged, err);
  if (!chassis_obj->is_object()) {
    err = "'chassis' must be an object";
    return false;
  }
  return merge_chassis_keys(*chassis_obj, merged, err);
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

bool load_arm_config(const Json* node, DriveProfile::ArmParams& out,
 std::string& err) {
 if (node == nullptr) return true;
 if (!node->is_object()) {
 err = "'arm' must be an object";
 return false;
 }
 const Json* enabled = node->find("enabled");
 if (enabled == nullptr || !enabled->is_bool()) {
 err = "'arm.enabled' must be a boolean";
 return false;
 }
 DriveProfile::ArmParams parsed;
 parsed.enabled = enabled->as_bool();
 if (!parsed.enabled) {
 out = parsed;
 return true;
 }
 const Json* base = node->find("base_xyz_m");
 if (base == nullptr || !base->is_array() || base->as_array().size() != 3) {
 err = "'arm.base_xyz_m' must be a 3-number array";
 return false;
 }
 for (int i = 0; i < 3; ++i) {
 const Json& value = base->as_array()[static_cast<std::size_t>(i)];
 if (!value.is_num() || !std::isfinite(value.as_num())) {
 err = "'arm.base_xyz_m' must contain finite numbers";
 return false;
 }
 if (i == 0) parsed.base_x_m = value.as_num();
 if (i == 1) parsed.base_y_m = value.as_num();
 if (i == 2) parsed.base_z_m = value.as_num();
 }
 const Json* links = node->find("links");
 if (links == nullptr || !links->is_array() || links->as_array().size() != 4) {
 err = "'arm.links' must contain exactly 4 entries";
 return false;
 }
 for (std::size_t i = 0; i < 4; ++i) {
 const Json& link = links->as_array()[i];
 if (!link.is_object()) {
 err = "'arm.links' entries must be objects";
 return false;
 }
 const char* keys[] = {"length_m", "mass_kg", "radius_m",
 "initial_angle_rad", "torque_limit_Nm", "damping_Nms"};
 double* slots[] = {&parsed.link_length_m[i], &parsed.link_mass_kg[i],
 &parsed.link_radius_m[i], &parsed.initial_angle_rad[i],
 &parsed.joint_torque_limit_Nm[i], &parsed.joint_damping_Nms[i]};
 for (int field = 0; field < 6; ++field) {
 const Json* value = link.find(keys[field]);
 if (value == nullptr || !value->is_num() ||
 !std::isfinite(value->as_num())) {
 err = std::string("'arm.links.") + std::to_string(i) + "." +
 keys[field] + "' must be a finite number";
 return false;
 }
 *slots[field] = value->as_num();
 }
 if (!(parsed.link_length_m[i] > 0.0) ||
 !(parsed.link_mass_kg[i] > 0.0) || !(parsed.link_radius_m[i] > 0.0) ||
 !(parsed.joint_torque_limit_Nm[i] > 0.0) ||
 !(parsed.joint_damping_Nms[i] > 0.0)) {
 err = std::string("'arm.links.") + std::to_string(i) +
 "' needs positive geometry, torque_limit_Nm and damping_Nms";
 return false;
 }
 const Json* axis = link.find("joint_axis");
 if (axis == nullptr || !axis->is_array() || axis->as_array().size() != 3) {
 err = std::string("'arm.links.") + std::to_string(i) +
 ".joint_axis' must be a 3-number array";
 return false;
 }
 double norm_sq = 0.0;
 for (int component = 0; component < 3; ++component) {
 const Json& value = axis->as_array()[static_cast<std::size_t>(component)];
 if (!value.is_num() || !std::isfinite(value.as_num())) {
 err = std::string("'arm.links.") + std::to_string(i) +
 ".joint_axis' must contain finite numbers";
 return false;
 }
 parsed.joint_axis[i][static_cast<std::size_t>(component)] = value.as_num();
 norm_sq += value.as_num() * value.as_num();
 }
 if (!(norm_sq > 1e-12) || !std::isfinite(norm_sq)) {
 err = std::string("'arm.links.") + std::to_string(i) +
 ".joint_axis' must have non-zero length";
 return false;
 }
 }
 out = parsed;
 return true;
}

bool load_robot_config(const std::string& path, RobotConfig& config,
                       std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = core::json_parse(text, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid robot config: " + parse_error;
    return false;
  }
  const Json& obj = *root;
  RobotConfig out;
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
  if (!DriveProfile::for_chassis(want, out.profile)) {
    err = "unknown chassis kind: " + want;
    return false;
  }
  if (!merge_drive_profile_keys(obj, out.profile, err)) return false;
  const Json* drive = obj.find("drive");
  if (drive != nullptr) {
    if (!drive->is_object()) {
      err = "'drive' must be an object";
      return false;
    }
    if (!merge_drive_profile_keys(*drive, out.profile, err)) return false;
  }
  // Present block merges, absent block validates the defaulted geometry
  // against the merged wheel radius — one call, both paths.
  if (!merge_chassis_config(obj.find("chassis"), out.profile, err)) {
    return false;
  }
 if (!load_arm_config(obj.find("arm"), out.profile.arm, err)) {
 return false;
 }
  const Json* lim = obj.find("limits");
 bool lateral_limit_present = false;
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
    Slot slots[] = {{"max_forward_mps", &out.limits.max_forward_mps},
                    {"max_yaw_rate_radps", &out.limits.max_yaw_rate_radps},
                    {"footprint_radius_m", &out.limits.footprint_radius_m},
                    {"max_steer_rad", &out.limits.max_steer_rad},
                    {"max_lateral_mps", &out.limits.max_lateral_mps}};
    for (const auto& s : slots) {
      if (!take_num(*lim, s.key, v, present, err)) return false;
      if (present) {
        if (!std::isfinite(v) || v < 0.0) {
          err = std::string("limits.'") + s.key +
                "' must be finite non-negative";
          return false;
        }
        *s.slot = v;
 if (std::string(s.key) == "max_lateral_mps") {
 lateral_limit_present = present;
 }
      }
    }
 if (!lateral_limit_present &&
 (want == "swerve" || want == "mecanum")) {
 // Holonomic bases are laterally capable by default. A machine file may
 // still set max_lateral_mps to zero deliberately to disable strafing.
 out.limits.max_lateral_mps = out.limits.max_forward_mps;
 }
  }
 const double body_corner_radius = std::hypot(
 out.profile.body_half_length_m, out.profile.body_half_width_m);
 if (out.limits.footprint_radius_m + 1e-12 < body_corner_radius) {
 err = "limits.'footprint_radius_m' must enclose the chassis body";
 return false;
 }
 const Json* navigation = obj.find("navigation");
 if (navigation != nullptr) {
 if (!navigation->is_object()) {
 err = "'navigation' must be an object";
 return false;
 }
 double value = 0.0;
 bool present = false;
 struct NavigationSlot {
 const char* key;
 double* slot;
 };
 NavigationSlot slots[] = {
 {"clearance_margin_m", &out.navigation.clearance_margin_m},
 {"soft_inflation_radius_m", &out.navigation.soft_inflation_radius_m}};
 for (const auto& slot : slots) {
 if (!take_num(*navigation, slot.key, value, present, err)) return false;
 if (!present) continue;
 if (!std::isfinite(value) || value < 0.0) {
 err = std::string("navigation.'") + slot.key +
 "' must be finite non-negative";
 return false;
 }
 *slot.slot = value;
 }
 }

 const Json* arrival = obj.find("arrival");
 if (arrival != nullptr) {
 if (!arrival->is_object()) {
 err = "'arrival' must be an object";
 return false;
 }
 double value = 0.0;
 bool present = false;
 struct ArrivalSlot {
 const char* key;
 double* slot;
 bool allow_zero;
 };
 ArrivalSlot slots[] = {
 {"position_tolerance_m", &out.arrival.position_tolerance_m, false},
 {"heading_tolerance_rad", &out.arrival.heading_tolerance_rad, false},
 {"maximum_position_sigma_m",
 &out.arrival.maximum_position_sigma_m, false},
 {"maximum_heading_sigma_rad",
 &out.arrival.maximum_heading_sigma_rad, false},
 {"maximum_forward_speed_mps",
 &out.arrival.maximum_forward_speed_mps, true},
 {"maximum_yaw_rate_radps",
 &out.arrival.maximum_yaw_rate_radps, true}};
 for (const auto& slot : slots) {
 if (!take_num(*arrival, slot.key, value, present, err)) return false;
 if (!present) continue;
 if (!std::isfinite(value) ||
 (slot.allow_zero ? value < 0.0 : value <= 0.0)) {
 err = std::string("arrival.'") + slot.key +
 (slot.allow_zero ? "' must be finite non-negative"
 : "' must be finite positive");
 return false;
 }
 *slot.slot = value;
 }
 if (!take_num(*arrival, "confidence_probability", value, present, err)) {
 return false;
 }
 if (present) {
 if (!std::isfinite(value) || value <= 0.0 || value >= 1.0) {
 err = "arrival.'confidence_probability' must be in (0, 1)";
 return false;
 }
 out.arrival.confidence_probability = value;
 }
 const Json* settle = arrival->find("settle_cycles");
 if (settle != nullptr) {
 if (!settle->is_num() || settle->as_num() <= 0.0 ||
 settle->as_num() != std::floor(settle->as_num())) {
 err = "arrival.'settle_cycles' must be a positive integer";
 return false;
 }
 out.arrival.settle_cycles = static_cast<int>(settle->as_num());
 }
 }

 const Json* motion = obj.find("motion");
 if (motion != nullptr) {
 if (!motion->is_object()) {
 err = "'motion' must be an object";
 return false;
 }
 double value = 0.0;
 bool present = false;
 struct MotionSlot {
 const char* key;
 double* slot;
 bool positive;
 };
 MotionSlot slots[] = {
 {"acceleration_mps2", &out.motion.acceleration_mps2, true},
 {"brake_deceleration_mps2",
 &out.motion.brake_deceleration_mps2, true},
 {"lateral_acceleration_mps2",
 &out.motion.lateral_acceleration_mps2, true},
 {"yaw_acceleration_radps2",
 &out.motion.yaw_acceleration_radps2, true},
 {"actuator_response_delay_s",
 &out.motion.actuator_response_delay_s, false},
 {"safety_reaction_time_s",
 &out.motion.safety_reaction_time_s, false},
 {"execution_fault_confirmation_s",
 &out.motion.execution_fault_confirmation_s, false}};
 for (const auto& slot : slots) {
 if (!take_num(*motion, slot.key, value, present, err)) return false;
 if (!present) continue;
 if (!std::isfinite(value) ||
 (slot.positive ? value <= 0.0 : value < 0.0)) {
 err = std::string("motion.'") + slot.key +
 (slot.positive ? "' must be finite positive"
 : "' must be finite non-negative");
 return false;
 }
 *slot.slot = value;
 }
 }
 // The motion block is a guaranteed envelope, not an aspirational tuning
 // value. Refuse contracts that the declared motor/traction/mass plant cannot
 // meet even with two driven contacts. Four-wheel bases retain extra margin.
 constexpr double kGravityMps2 = 9.80665;
 const double declared_mass_kg =
 out.profile.vehicle_mass_kg + out.profile.payload_mass_kg;
 const double traction_limit_mps2 =
 out.profile.tire_friction_mu * kGravityMps2;
 const double drive_limit_mps2 =
 2.0 * out.profile.stall_force_N / declared_mass_kg;
 const double longitudinal_limit_mps2 =
 std::min(traction_limit_mps2, drive_limit_mps2);
 if (out.motion.acceleration_mps2 > longitudinal_limit_mps2 + 1e-9 ||
 out.motion.brake_deceleration_mps2 >
 longitudinal_limit_mps2 + 1e-9) {
 err = "motion longitudinal capability exceeds declared motor/traction plant";
 return false;
 }
 if (out.motion.lateral_acceleration_mps2 >
 traction_limit_mps2 + 1e-9) {
 err = "motion lateral capability exceeds declared tire traction";
 return false;
 }
  const Json* sens = obj.find("sensors");
  if (sens == nullptr) {
    out.sensors = default_sensor_mounts();  // bare file keeps the mast
  } else {
    if (!sens->is_array()) {
      err = "'sensors' must be an array";
      return false;
    }
    for (const auto& e : sens->as_array()) {
      if (!e.is_object()) {
        err = "sensor entry must be an object";
        return false;
      }
      SensorMount m;
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
  if (links == nullptr) {
    out.links = default_link_mounts();  // bare file keeps the mast
  } else {
    if (!links->is_array()) {
      err = "'links' must be an array";
      return false;
    }
    for (const auto& e : links->as_array()) {
      if (!e.is_object()) {
        err = "link entry must be an object";
        return false;
      }
      LinkMount m;
      std::string link, parent;
      bool pl = false, pp = false;
      if (!take_str(e, "link", link, pl, err) || !pl || link.empty()) {
        err = "link entry needs a non-empty 'link'";
        return false;
      }
      if (!take_str(e, "parent", parent, pp, err) || !pp || parent.empty()) {
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
    Slot slots[] = {{"max_torque_Nm", &out.brake_max_torque_Nm},
                    {"apply_time_s", &out.brake_apply_time_s},
                    {"release_time_s", &out.brake_release_time_s}};
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
 if (!(out.brake_apply_time_s > 0.0) ||
 !(out.brake_release_time_s > 0.0)) {
 err = "brake apply/release times must be finite and positive";
 return false;
 }
  }
 std::string planner;
 bool planner_present = false;
 if (!take_str(obj, "local_planner", planner, planner_present, err)) {
 return false;
 }
 if (planner_present) {
 if (planner != "dwa" && planner != "dwb" && planner != "teb") {
 err = "'local_planner' must be 'dwa', 'dwb' or 'teb'";
 return false;
 }
 out.local_planner_kind = planner;
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

RobotDescription description_from_config(const RobotConfig& config) {
  RobotDescription desc;
  desc.robot_id = config.robot_id;
  desc.version = config.version;
  desc.links = config.links;
  desc.sensors = config.sensors;
  desc.limits = config.limits;
 desc.navigation = config.navigation;
 desc.arrival = config.arrival;
 desc.motion = config.motion;
 desc.chassis_kind = config.profile.chassis_kind;
 desc.local_planner_kind = config.local_planner_kind;
 if (desc.chassis_kind == "differential") {
   desc.limits.motion_model = ChassisMotionModel::kDifferential;
 } else if (desc.chassis_kind == "ackermann") {
   desc.limits.motion_model = ChassisMotionModel::kAckermann;
 } else if (desc.chassis_kind == "mecanum" ||
            desc.chassis_kind == "swerve") {
   desc.limits.motion_model = ChassisMotionModel::kHolonomic;
 } else if (desc.chassis_kind == "tracked") {
   desc.limits.motion_model = ChassisMotionModel::kTracked;
 }
  // Declared plant loadout for autonomy envelopes (description, never
  // truth): COM folds the declared payload master-style (weighted
  // average; payload at chassis COM when unset = pure ballast). The SIM
  // plant folds the identical declared numbers engine-side.
  {
    const auto& p = config.profile;
    const double com_eff =
        p.com_height_m > 0.0 ? p.com_height_m : p.wheel_radius_m;
    const double pcom =
        p.payload_height_m > 0.0 ? p.payload_height_m : com_eff;
    desc.plant.wheel_radius_m = p.wheel_radius_m;
    desc.plant.track_width_m = p.track_width_m;
    desc.plant.wheel_base_m = p.wheel_base_m;
 desc.plant.body_half_length_m = p.body_half_length_m;
 desc.plant.body_half_width_m = p.body_half_width_m;
 desc.plant.body_half_height_m = p.body_half_height_m;
    desc.plant.total_mass_kg = p.vehicle_mass_kg + p.payload_mass_kg;
    desc.plant.com_height_m =
        (p.vehicle_mass_kg * com_eff + p.payload_mass_kg * pcom) /
        std::max(0.5, desc.plant.total_mass_kg);
  }
  return desc;
}

std::optional<RobotConfig> load_default_robot_config(
    const std::string& explicit_path, std::string* used_path,
    std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  RobotConfig out;
  if (!explicit_path.empty()) {
    std::string load_err;
    if (!load_robot_config(explicit_path, out, &load_err)) {
      err = load_err;
      return std::nullopt;
    }
    if (used_path != nullptr) *used_path = explicit_path;
    err.clear();
    return out;
  }
  constexpr char kSeed[] = "configs/robot/differential.json";
  std::ifstream probe(kSeed, std::ios::binary);
  if (probe.is_open()) {
    probe.close();
    std::string load_err;
    if (!load_robot_config(kSeed, out, &load_err)) {
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

}  // namespace lost_signal::robot::platform
