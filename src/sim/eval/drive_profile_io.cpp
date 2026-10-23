#include "sim/eval/drive_profile_io.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"

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
  // One contract: the machine file and legacy flat profiles share the
  // identical merge (drive keys + chassis object). Flat files predate the
  // chassis block; absent = documented defaults, never zeros.
  if (!robot::platform::merge_drive_profile_keys(obj, merged, err)) {
    return false;
  }
  if (!robot::platform::merge_chassis_config(obj.find("chassis"), merged,
                                             err)) {
    return false;
  }
  profile = merged;
  return true;
}

}  // namespace lost_signal::sim::eval
