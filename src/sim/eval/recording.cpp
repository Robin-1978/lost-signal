#include "sim/eval/recording.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdio>
#include <vector>

#include "robot/core/json.hpp"

namespace lost_signal::sim::eval {
using robot::core::Json;
using robot::core::json_dump;
using robot::core::json_parse;

namespace {

Json::Array floats_to_json(const std::vector<float>& values) {
  Json::Array out;
  out.reserve(values.size());
  for (float v : values) out.push_back(Json::Num(static_cast<double>(v)));
  return out;
}

bool floats_from_json(const Json* node, std::vector<float>& out,
                      std::string& error) {
  if (node == nullptr || !node->is_array()) {
    error = "expected float array";
    return false;
  }
  out.clear();
  out.reserve(node->as_array().size());
  for (const auto& item : node->as_array()) {
    if (!item.is_num()) {
      error = "non-numeric array element";
      return false;
    }
    out.push_back(static_cast<float>(item.as_num()));
  }
  return true;
}

const Json* req(const Json& obj, const char* key, std::string& error) {
  const Json* node = obj.find(key);
  if (node == nullptr) {
    error = std::string("missing key '") + key + "'";
    return nullptr;
  }
  return node;
}

bool req_num(const Json& obj, const char* key, double& out,
             std::string& error) {
  const Json* node = req(obj, key, error);
  if (node == nullptr || !node->is_num()) {
    if (node != nullptr) error = std::string("key '") + key + "' not a number";
    return false;
  }
  out = node->as_num();
  return true;
}

bool req_bool(const Json& obj, const char* key, bool& out,
              std::string& error) {
  const Json* node = req(obj, key, error);
  if (node == nullptr || !node->is_bool()) {
    if (node != nullptr)
      error = std::string("key '") + key + "' not a bool";
    return false;
  }
  out = node->as_bool();
  return true;
}

}  // namespace

std::string step_to_json_line(const RecordedStep& step) {
  Json::Object o;
  o.emplace_back("v", Json::Num(2));
  o.emplace_back("seq", Json::Num(static_cast<double>(step.sequence)));
  o.emplace_back("t", Json::Num(static_cast<double>(step.stamp_ns)));
  o.emplace_back("src", Json::Str(step.source));
  o.emplace_back("ranges", Json::Arr(floats_to_json(step.ranges_m)));
 if (step.intensity_valid) {
 o.emplace_back("intensity_ref", Json::Arr(floats_to_json(step.intensities)));
 } else {
 o.emplace_back("intensity_ref", Json::Null());
 }
 if (step.time_offset_valid) {
 o.emplace_back("time_offsets", Json::Arr(floats_to_json(step.time_offsets_s)));
 } else {
 o.emplace_back("time_offsets", Json::Null());
 }
  if (step.scan_static_valid) {
    o.emplace_back("static_ref",
                   Json::Arr(floats_to_json(step.static_ranges_m)));
  } else {
    o.emplace_back("static_ref", Json::Null());
  }
  o.emplace_back("scan_ok", Json::Bool(step.scan_valid));
  o.emplace_back("angle_min", Json::Num(step.angle_min_rad));
  o.emplace_back("angle_inc", Json::Num(step.angle_increment_rad));
  Json::Object odom;
  odom.emplace_back("valid", Json::Bool(step.odom_valid));
  odom.emplace_back("fwd", Json::Num(step.forward_mps));
  odom.emplace_back("yaw", Json::Num(step.yaw_rate_radps));
  o.emplace_back("odom", Json::Obj(std::move(odom)));
  Json::Object imu;
  imu.emplace_back("valid", Json::Bool(step.imu_valid));
  Json::Array gyro;
  gyro.push_back(Json::Num(step.gyro_x));
  gyro.push_back(Json::Num(step.gyro_y));
  gyro.push_back(Json::Num(step.gyro_z));
  imu.emplace_back("gyro", Json::Arr(std::move(gyro)));
  Json::Array acc;
  acc.push_back(Json::Num(step.ax));
  acc.push_back(Json::Num(step.ay));
  acc.push_back(Json::Num(step.az));
  imu.emplace_back("acc", Json::Arr(std::move(acc)));
  o.emplace_back("imu", Json::Obj(std::move(imu)));
  Json::Array drives;
  for (double d : step.drives_radps) drives.push_back(Json::Num(d));
  o.emplace_back("drives", Json::Arr(std::move(drives)));
  Json::Object steers;
  Json::Array spos, svel;
  for (double s : step.steers_rad) spos.push_back(Json::Num(s));
  for (double s : step.steers_vel_radps) svel.push_back(Json::Num(s));
  steers.emplace_back("pos", Json::Arr(std::move(spos)));
  steers.emplace_back("vel", Json::Arr(std::move(svel)));
  o.emplace_back("steers", Json::Obj(std::move(steers)));
  return json_dump(Json::Obj(std::move(o)));
}

bool step_from_json_line(const std::string& line, RecordedStep& step,
                         std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string parse_error;
  auto root = json_parse(line, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid step line: " + parse_error;
    return false;
  }
  const Json& obj = *root;
  double v = 0, seq = 0, stamp = 0;
  if (!req_num(obj, "v", v, err) || v != 2.0) {
    if (err.empty()) err = "unsupported schema version";
    return false;
  }
  if (!req_num(obj, "seq", seq, err) || seq < 0.0 ||
      !req_num(obj, "t", stamp, err)) {
    return false;
  }
  step.sequence = static_cast<std::uint64_t>(seq);
  step.stamp_ns = static_cast<std::int64_t>(stamp);
  const Json* src = req(obj, "src", err);
  if (src == nullptr || !src->is_str()) {
    if (src != nullptr) err = "key 'src' not a string";
    return false;
  }
  step.source = src->as_str();
  const Json* ranges = req(obj, "ranges", err);
  if (!floats_from_json(ranges, step.ranges_m, err)) return false;
 step.intensity_valid = false;
 step.intensities.clear();
 if (const Json* intensity = obj.find("intensity_ref"); intensity != nullptr &&
 !intensity->is_null()) {
 if (!floats_from_json(intensity, step.intensities, err)) return false;
 step.intensity_valid = true;
 }
 step.time_offset_valid = false;
 step.time_offsets_s.clear();
 if (const Json* offsets = obj.find("time_offsets"); offsets != nullptr &&
 !offsets->is_null()) {
 if (!floats_from_json(offsets, step.time_offsets_s, err)) return false;
 step.time_offset_valid = true;
 }
  const Json* sref = req(obj, "static_ref", err);
  if (sref == nullptr) return false;
  if (sref->is_null()) {
    step.scan_static_valid = false;
    step.static_ranges_m.clear();
  } else if (!floats_from_json(sref, step.static_ranges_m, err)) {
    return false;
  } else {
    step.scan_static_valid = true;
  }
  if (!req_bool(obj, "scan_ok", step.scan_valid, err)) return false;
  if (!req_num(obj, "angle_min", step.angle_min_rad, err)) return false;
  if (!req_num(obj, "angle_inc", step.angle_increment_rad, err)) return false;
  const Json* odom = req(obj, "odom", err);
  if (odom == nullptr || !odom->is_object()) {
    if (odom != nullptr) err = "key 'odom' not an object";
    return false;
  }
  if (!req_bool(*odom, "valid", step.odom_valid, err)) return false;
  if (!req_num(*odom, "fwd", step.forward_mps, err)) return false;
  if (!req_num(*odom, "yaw", step.yaw_rate_radps, err)) return false;
  const Json* imu = req(obj, "imu", err);
  if (imu == nullptr || !imu->is_object()) {
    if (imu != nullptr) err = "key 'imu' not an object";
    return false;
  }
  if (!req_bool(*imu, "valid", step.imu_valid, err)) return false;
  const Json* gyro = req(*imu, "gyro", err);
  const Json* acc = req(*imu, "acc", err);
  auto triple = [&](const Json* node, const char* name, double& x, double& y,
                    double& z) {
    if (node == nullptr || !node->is_array() ||
        node->as_array().size() != 3) {
      if (node != nullptr) err = std::string("key '") + name + "' not a triple";
      return false;
    }
    const auto& a = node->as_array();
    if (!a[0].is_num() || !a[1].is_num() || !a[2].is_num()) {
      err = std::string("key '") + name + "' has non-numeric elements";
      return false;
    }
    x = a[0].as_num();
    y = a[1].as_num();
    z = a[2].as_num();
    return true;
  };
  if (!triple(gyro, "gyro", step.gyro_x, step.gyro_y, step.gyro_z)) return false;
  if (!triple(acc, "acc", step.ax, step.ay, step.az)) return false;
  const Json* drives = req(obj, "drives", err);
  if (drives == nullptr || !drives->is_array() ||
      drives->as_array().size() > 4) {
    if (drives != nullptr) err = "key 'drives' must be an array of <= 4 numbers";
    return false;
  }
  step.drives_radps.clear();
  for (const auto& item : drives->as_array()) {
    if (!item.is_num()) {
      err = "non-numeric drives element";
      return false;
    }
    step.drives_radps.push_back(item.as_num());
  }
  const Json* steers = req(obj, "steers", err);
  if (steers == nullptr || !steers->is_object()) {
    if (steers != nullptr) err = "key 'steers' not an object";
    return false;
  }
  const Json* spos = req(*steers, "pos", err);
  const Json* svel = req(*steers, "vel", err);
  auto num_array = [&](const Json* node, const char* name,
                       std::vector<double>& out) {
    if (node == nullptr || !node->is_array() ||
        node->as_array().size() > 4) {
      if (node != nullptr)
        err = std::string("key '") + name + "' must be an array of <= 4 numbers";
      return false;
    }
    out.clear();
    for (const auto& item : node->as_array()) {
      if (!item.is_num()) {
        err = std::string("non-numeric '") + name + "' element";
        return false;
      }
      out.push_back(item.as_num());
    }
    return true;
  };
  if (!num_array(spos, "pos", step.steers_rad)) return false;
  if (!num_array(svel, "vel", step.steers_vel_radps)) return false;
  return true;
}

RecordedStep record_step(const robot::iface::ObservationFrame& obs) {
  RecordedStep s;
  s.sequence = obs.sequence;
  s.stamp_ns = obs.stamp_ns;
  s.source = obs.source;
  s.ranges_m = obs.ranges_m;
 s.intensities = obs.intensities;
 s.intensity_valid = obs.intensity_valid;
 s.time_offsets_s = obs.time_offsets_s;
 s.time_offset_valid = obs.time_offset_valid;
  s.static_ranges_m = obs.static_ranges_m;
  s.scan_valid = obs.scan_valid;
  s.scan_static_valid = obs.scan_static_valid;
  s.angle_min_rad = obs.scan_angle_min_rad;
  s.angle_increment_rad = obs.scan_angle_increment_rad;
  s.odom_valid = obs.odometry_valid;
  s.forward_mps = obs.forward_mps;
  s.yaw_rate_radps = obs.yaw_rate_radps;
  s.imu_valid = obs.imu_valid;
  s.gyro_x = obs.imu_gyro_x_radps;
  s.gyro_y = obs.imu_gyro_y_radps;
  s.gyro_z = obs.imu_gyro_z_radps;
  s.ax = obs.imu_ax_mps2;
  s.ay = obs.imu_ay_mps2;
  s.az = obs.imu_az_mps2;
  s.drives_radps.assign(obs.drive_radps.begin(),
                        obs.drive_radps.begin() + obs.drive_count);
  s.steers_rad.assign(obs.steer_rad.begin(),
                      obs.steer_rad.begin() + obs.steer_count);
  s.steers_vel_radps.assign(obs.steer_vel_radps.begin(),
                            obs.steer_vel_radps.begin() + obs.steer_count);
  return s;
}

robot::iface::ObservationFrame step_to_observation(const RecordedStep& step) {
  robot::iface::ObservationFrame o;
  o.sequence = step.sequence;
  o.stamp_ns = step.stamp_ns;
  o.source = step.source;
  o.frame = robot::core::Frame::kUnknown;
  o.ranges_m = step.ranges_m;
  o.scan_valid = step.scan_valid;
 o.intensities = step.intensities;
 o.intensity_valid = step.intensity_valid;
 o.time_offsets_s = step.time_offsets_s;
 o.time_offset_valid = step.time_offset_valid;
  o.static_ranges_m = step.static_ranges_m;
  o.scan_static_valid = step.scan_static_valid;
  o.scan_angle_min_rad = step.angle_min_rad;
  o.scan_angle_increment_rad = step.angle_increment_rad;
  o.odometry_valid = step.odom_valid;
  o.forward_mps = step.forward_mps;
  o.yaw_rate_radps = step.yaw_rate_radps;
  o.imu_valid = step.imu_valid;
  o.imu_gyro_x_radps = step.gyro_x;
  o.imu_gyro_y_radps = step.gyro_y;
  o.imu_gyro_z_radps = step.gyro_z;
  o.imu_ax_mps2 = step.ax;
  o.imu_ay_mps2 = step.ay;
  o.imu_az_mps2 = step.az;
  o.drive_count = static_cast<std::uint8_t>(std::min<std::size_t>(step.drives_radps.size(), 4));
  o.steer_count = static_cast<std::uint8_t>(std::min<std::size_t>(step.steers_rad.size(), 4));
  for (std::size_t i = 0; i < o.drive_count; ++i) o.drive_radps[i] = step.drives_radps[i];
  for (std::size_t i = 0; i < o.steer_count; ++i) {
    o.steer_rad[i] = step.steers_rad[i];
    if (i < step.steers_vel_radps.size()) o.steer_vel_radps[i] = step.steers_vel_radps[i];
  }
  return o;
}

}  // namespace lost_signal::sim::eval
