#include "sim/eval/world_io.hpp"

#include <fstream>
#include <sstream>

#include "sim/eval/json.hpp"

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

bool write_text(const std::string& path, const std::string& text,
                std::string* error) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    if (error) *error = "cannot write file: " + path;
    return false;
  }
  out << text;
  out.flush();
  if (!out.good()) {
    if (error) *error = "write failed: " + path;
    return false;
  }
  return true;
}

Json box_to_json(const world::StaticBox& b) {
  return Json::Obj({{"x0", Json::Num(b.x0)},
                    {"y0", Json::Num(b.y0)},
                    {"x1", Json::Num(b.x1)},
                    {"y1", Json::Num(b.y1)},
                    {"height_m", Json::Num(b.height_m)}});
}

bool box_from_json(const Json& j, world::StaticBox& b, std::string& err) {
  if (!j.is_object()) {
    err = "box not an object";
    return false;
  }
  for (const char* k : {"x0", "y0", "x1", "y1", "height_m"}) {
    const Json* n = j.find(k);
    if (n == nullptr || !n->is_num()) {
      err = std::string("box key '") + k + "' missing or not a number";
      return false;
    }
  }
  b.x0 = j.find("x0")->as_num();
  b.y0 = j.find("y0")->as_num();
  b.x1 = j.find("x1")->as_num();
  b.y1 = j.find("y1")->as_num();
  b.height_m = j.find("height_m")->as_num();
  return true;
}

Json place_to_json(const world::NamedPlace& p) {
  return Json::Obj({{"place_id", Json::Str(p.place_id)},
                    {"x_m", Json::Num(p.x_m)},
                    {"y_m", Json::Num(p.y_m)},
                    {"yaw_rad", Json::Num(p.yaw_rad)}});
}

bool place_from_json(const Json& j, world::NamedPlace& p, std::string& err) {
  if (!j.is_object()) {
    err = "place not an object";
    return false;
  }
  const Json* id = j.find("place_id");
  if (id == nullptr || !id->is_str() || id->as_str().empty()) {
    err = "place_id missing or empty";
    return false;
  }
  for (const char* k : {"x_m", "y_m", "yaw_rad"}) {
    const Json* n = j.find(k);
    if (n == nullptr || !n->is_num()) {
      err = std::string("place key '") + k + "' missing or not a number";
      return false;
    }
  }
  p.place_id = id->as_str();
  p.x_m = j.find("x_m")->as_num();
  p.y_m = j.find("y_m")->as_num();
  p.yaw_rad = j.find("yaw_rad")->as_num();
  return true;
}

}  // namespace

bool save_world_file(const std::string& path,
                     const std::vector<world::StaticBox>& boxes,
                     const std::vector<world::NamedPlace>& places,
                     std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  Json::Array box_arr;
  for (const auto& b : boxes) box_arr.push_back(box_to_json(b));
  Json::Array place_arr;
  for (const auto& p : places) place_arr.push_back(place_to_json(p));
  const Json root = Json::Obj({{"format", Json::Str(kWorldFileFormat)},
                               {"boxes", Json::Arr(std::move(box_arr))},
                               {"places", Json::Arr(std::move(place_arr))}});
  if (!write_text(path, json_dump(root), error)) return false;
  err.clear();
  return true;
}

bool load_world_file(const std::string& path,
                     std::vector<world::StaticBox>& boxes,
                     std::vector<world::NamedPlace>& places,
                     std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = json_parse(text, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid world file: " + parse_error;
    return false;
  }
  const Json* fmt = root->find("format");
  if (fmt == nullptr || !fmt->is_str() ||
      fmt->as_str() != kWorldFileFormat) {
    err = "unsupported world format";
    return false;
  }
  const Json* bj = root->find("boxes");
  const Json* pj = root->find("places");
  if (bj == nullptr || !bj->is_array() || pj == nullptr ||
      !pj->is_array()) {
    err = "world file needs boxes[] and places[]";
    return false;
  }
  std::vector<world::StaticBox> b_out;
  for (const auto& e : bj->as_array()) {
    world::StaticBox b;
    if (!box_from_json(e, b, err)) return false;
    b_out.push_back(b);
  }
  std::vector<world::NamedPlace> p_out;
  for (const auto& e : pj->as_array()) {
    world::NamedPlace p;
    if (!place_from_json(e, p, err)) return false;
    p_out.push_back(p);
  }
  boxes = std::move(b_out);
  places = std::move(p_out);
  err.clear();
  return true;
}

}  // namespace lost_signal::sim::eval
