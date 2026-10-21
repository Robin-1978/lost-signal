#include "robot/platform/robot_drive_profile.hpp"

#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <unordered_set>

namespace sim {
namespace {

using json = nlohmann::json;

void reject_unknown_keys(const json& object, const std::string& source,
 const std::string& section,
 std::initializer_list<const char*> allowed) {
 if (!object.is_object()) {
 throw std::runtime_error(source + ": '" + section + "' must be an object");
 }
 std::unordered_set<std::string> known;
 for (const char* key : allowed) {
 known.emplace(key);
 }
 for (const auto& item : object.items()) {
 if (!known.count(item.key())) {
 throw std::runtime_error(source + ": unknown key '" + section + "." + item.key() + "'");
 }
 }
}

double required_number(const json& object, const std::string& source, const std::string& section,
 const char* key) {
 const auto found = object.find(key);
 if (found == object.end()) {
 throw std::runtime_error(source + ": missing key '" + section + "." + key + "'");
 }
 if (!found->is_number()) {
 throw std::runtime_error(source + ": '" + section + "." + key + "' must be a number");
 }
 return found->get<double>();
}

double optional_number(const json& object, const std::string& source, const std::string& section,
 const char* key, double fallback) {
 const auto found = object.find(key);
 if (found == object.end()) {
 return fallback;
 }
 if (!found->is_number()) {
 throw std::runtime_error(source + ": '" + section + "." + key + "' must be a number");
 }
 return found->get<double>();
}

std::string required_string(const json& object, const std::string& source,
 const std::string& section, const char* key) {
 const auto found = object.find(key);
 if (found == object.end()) {
 throw std::runtime_error(source + ": missing key '" + section + "." + key + "'");
 }
 if (!found->is_string()) {
 throw std::runtime_error(source + ": '" + section + "." + key + "' must be a string");
 }
 return found->get<std::string>();
}

bool positive(double value) { return std::isfinite(value) && value > 0.0; }
bool non_negative(double value) { return std::isfinite(value) && value >= 0.0; }

} // namespace

RobotDriveProfile parse_robot_drive(const json& drive, const std::string& source) {
 reject_unknown_keys(drive, source, "drive",
 {"type", "wheel_radius_m", "wheel_base_m", "rated_speed_mps",
 "effective_track_width_m", "mass_kg", "yaw_inertia_kgm2",
 "max_yaw_accel_radps2", "max_yaw_rate_rads", "track_length_m",
 "track_width_m", "motor", "steering", "energy", "encoder",
 "service_brake"});
 if (!drive.contains("motor") || !drive.contains("steering") || !drive.contains("energy") ||
 !drive.contains("encoder")) {
 throw std::runtime_error(source + ": drive block is missing motor, steering, energy or encoder");
 }

 RobotDriveProfile profile;
 profile.drive_type = required_string(drive, source, "drive", "type");
 profile.wheel_radius_m = required_number(drive, source, "drive", "wheel_radius_m");
 profile.wheel_base_m = required_number(drive, source, "drive", "wheel_base_m");
 profile.rated_speed_mps = required_number(drive, source, "drive", "rated_speed_mps");
 profile.effective_track_width_m =
 optional_number(drive, source, "drive", "effective_track_width_m", profile.wheel_base_m);
 profile.mass_kg = required_number(drive, source, "drive", "mass_kg");
 profile.yaw_inertia_kgm2 = required_number(drive, source, "drive", "yaw_inertia_kgm2");
 profile.max_yaw_accel_radps2 = required_number(drive, source, "drive", "max_yaw_accel_radps2");
 profile.tracked_max_yaw_rate_rads =
 optional_number(drive, source, "drive", "max_yaw_rate_rads", 1.5);
 profile.track_length_m = optional_number(drive, source, "drive", "track_length_m", 0.7);
 profile.track_width_m = optional_number(drive, source, "drive", "track_width_m", 0.58);

 const auto& motor = drive.at("motor");
 reject_unknown_keys(motor, source, "drive.motor",
 {"max_wheel_speed_mps", "max_wheel_accel_mps2", "stall_force_N",
 "free_speed_mps", "pwm_limit", "integral_limit", "wheel_equiv_mass_kg",
 "controller_bandwidth_radps", "controller_integral_time_s",
 "controller_derivative_time_s", "static_friction_feedforward_duty",
 "coulomb_smoothing_speed_mps", "viscous_drag_N_per_mps"});
 profile.motor.max_wheel_speed_mps =
 required_number(motor, source, "drive.motor", "max_wheel_speed_mps");
 profile.motor.max_wheel_accel_mps2 =
 required_number(motor, source, "drive.motor", "max_wheel_accel_mps2");
 profile.motor.stall_force_N = required_number(motor, source, "drive.motor", "stall_force_N");
 profile.motor.free_speed_mps = required_number(motor, source, "drive.motor", "free_speed_mps");
 profile.motor.pwm_limit = required_number(motor, source, "drive.motor", "pwm_limit");
 profile.motor.integral_limit = required_number(motor, source, "drive.motor", "integral_limit");
 profile.motor.wheel_equiv_mass_kg =
 required_number(motor, source, "drive.motor", "wheel_equiv_mass_kg");
 profile.motor.controller_bandwidth_radps =
 required_number(motor, source, "drive.motor", "controller_bandwidth_radps");
 profile.motor.controller_integral_time_s =
 required_number(motor, source, "drive.motor", "controller_integral_time_s");
 profile.motor.controller_derivative_time_s =
 required_number(motor, source, "drive.motor", "controller_derivative_time_s");
 profile.motor.static_friction_feedforward_duty =
 required_number(motor, source, "drive.motor", "static_friction_feedforward_duty");
 profile.motor.coulomb_smoothing_speed_mps =
 required_number(motor, source, "drive.motor", "coulomb_smoothing_speed_mps");
 profile.motor.viscous_drag_N_per_mps =
 required_number(motor, source, "drive.motor", "viscous_drag_N_per_mps");

 const auto& steering = drive.at("steering");
 reject_unknown_keys(steering, source, "drive.steering",
 {"integral_limit", "max_steer_rad", "max_steer_rate_radps",
 "servo_response_s", "stall_torque_Nm"});
 profile.steering.integral_limit =
 required_number(steering, source, "drive.steering", "integral_limit");
 profile.steering.max_steer_rad =
 optional_number(steering, source, "drive.steering", "max_steer_rad", 0.6);
 profile.steering.max_steer_rate_radps =
 optional_number(steering, source, "drive.steering", "max_steer_rate_radps", 2.5);
 profile.steering.servo_response_s =
 optional_number(steering, source, "drive.steering", "servo_response_s", 0.4);
 profile.steering.stall_torque_Nm =
 optional_number(steering, source, "drive.steering", "stall_torque_Nm", 8.0);

 const auto& energy = drive.at("energy");
 reject_unknown_keys(energy, source, "drive.energy", {"base_w", "per_v_w", "per_w_w"});
 profile.energy_base_w = required_number(energy, source, "drive.energy", "base_w");
 profile.energy_per_v_w = required_number(energy, source, "drive.energy", "per_v_w");
 profile.energy_per_w_w = required_number(energy, source, "drive.energy", "per_w_w");

 const auto& encoder = drive.at("encoder");
 reject_unknown_keys(encoder, source, "drive.encoder", {"counts_per_revolution"});
 const auto counts = encoder.find("counts_per_revolution");
 if (counts == encoder.end() || !counts->is_number_integer()) {
 throw std::runtime_error(source + ": 'drive.encoder.counts_per_revolution' must be an integer");
 }
 profile.encoder.counts_per_revolution = counts->get<int>();

 if (const auto brake = drive.find("service_brake"); brake != drive.end()) {
 reject_unknown_keys(*brake, source, "drive.service_brake",
 {"enabled", "maximum_torque_Nm", "apply_time_s", "release_time_s",
 "low_speed_smoothing_radps"});
 profile.service_brake.enabled = brake->at("enabled").get<bool>();
 profile.service_brake.maximum_torque_Nm = required_number(
 *brake, source, "drive.service_brake", "maximum_torque_Nm");
 profile.service_brake.apply_time_s =
 required_number(*brake, source, "drive.service_brake", "apply_time_s");
 profile.service_brake.release_time_s =
 required_number(*brake, source, "drive.service_brake", "release_time_s");
 profile.service_brake.low_speed_smoothing_radps = required_number(
 *brake, source, "drive.service_brake", "low_speed_smoothing_radps");
 }

 validate_robot_drive(profile, source);
 return profile;
}

void validate_robot_drive(const RobotDriveProfile& profile, const std::string& source) {
 if (profile.drive_type.empty()) {
 throw std::runtime_error(source + ": drive.type must be set");
 }
 if (!positive(profile.wheel_radius_m) || !positive(profile.wheel_base_m) ||
 !positive(profile.effective_track_width_m) || !positive(profile.rated_speed_mps)) {
 throw std::runtime_error(
 source + ": wheel radius, wheel base, effective track width and rated speed must be positive");
 }
 if (!positive(profile.mass_kg) || !positive(profile.yaw_inertia_kgm2) ||
 !non_negative(profile.max_yaw_accel_radps2)) {
 throw std::runtime_error(source + ": mass and yaw inertia must be positive and yaw "
 "acceleration non-negative");
 }
 if (!positive(profile.tracked_max_yaw_rate_rads) || !positive(profile.track_length_m) ||
 !positive(profile.track_width_m)) {
 throw std::runtime_error(source + ": track geometry must be positive");
 }
 const auto& motor = profile.motor;
 if (!positive(motor.max_wheel_speed_mps) || !positive(motor.max_wheel_accel_mps2) ||
 !positive(motor.stall_force_N) || !positive(motor.free_speed_mps) ||
 !positive(motor.wheel_equiv_mass_kg) || !positive(motor.controller_bandwidth_radps) ||
 !positive(motor.controller_integral_time_s) ||
 !non_negative(motor.controller_derivative_time_s)) {
 throw std::runtime_error(source + ": motor capability values must be positive");
 }
 if (!(std::isfinite(motor.pwm_limit) && motor.pwm_limit > 0.0 && motor.pwm_limit <= 1.0)) {
 throw std::runtime_error(source + ": motor.pwm_limit must lie in (0, 1]");
 }
 if (!non_negative(motor.static_friction_feedforward_duty) ||
 !non_negative(motor.coulomb_smoothing_speed_mps) || !non_negative(motor.viscous_drag_N_per_mps)) {
 throw std::runtime_error(source + ": motor drag and feedforward terms must be non-negative");
 }
 if (!positive(profile.steering.max_steer_rad) || !positive(profile.steering.max_steer_rate_radps) ||
 !positive(profile.steering.servo_response_s) ||
 !non_negative(profile.steering.stall_torque_Nm)) {
 throw std::runtime_error(source + ": steering geometry and response must be positive");
 }
 if (!non_negative(profile.energy_base_w) || !non_negative(profile.energy_per_v_w) ||
 !non_negative(profile.energy_per_w_w)) {
 throw std::runtime_error(source + ": energy coefficients must be non-negative");
 }
 if (profile.encoder.counts_per_revolution <= 0) {
 throw std::runtime_error(source + ": encoder counts_per_revolution must be positive");
 }
 const auto& brake = profile.service_brake;
 if (brake.enabled) {
 if (!positive(brake.maximum_torque_Nm)) {
 throw std::runtime_error(source + ": an enabled service brake must declare a maximum torque");
 }
 if (!non_negative(brake.apply_time_s) || !non_negative(brake.release_time_s) ||
 !positive(brake.low_speed_smoothing_radps)) {
 throw std::runtime_error(source + ": service brake timing must be finite");
 }
 }
}

} // namespace sim

