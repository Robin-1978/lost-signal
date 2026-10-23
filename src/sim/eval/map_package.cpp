#include "sim/eval/map_package.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::sim::eval {
using robot::core::Json;
using robot::core::json_dump;
using robot::core::json_parse;

namespace {

// FNV-1a 32-bit over the canonical content bytes (everything but the
// stored checksum itself). Integrity check, not a security boundary.
std::uint32_t fnv1a(const void* data, std::size_t len, std::uint32_t hash) {
  const auto* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < len; ++i) {
    hash ^= bytes[i];
    hash *= 16777619u;
  }
  return hash;
}

void mix_str(std::uint32_t& h, const std::string& s) {
  h = fnv1a(s.data(), s.size(), h);
}

void mix_num(std::uint32_t& h, double v) {
  static_assert(sizeof(double) == 8);
  std::uint64_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  h = fnv1a(&bits, sizeof(bits), h);
}

void mix_u32(std::uint32_t& h, std::uint32_t v) {
  h = fnv1a(&v, sizeof(v), h);
}

std::uint32_t package_checksum(const MapPackage& p) {
  std::uint32_t h = 2166136261u;
  mix_str(h, p.map_id);
  mix_num(h, p.resolution_m);
  mix_num(h, p.origin_x_m);
  mix_num(h, p.origin_y_m);
  mix_u32(h, p.width);
  mix_u32(h, p.height);
  for (int8_t c : p.occupancy) {
    h = fnv1a(&c, sizeof(c), h);
  }
  for (const auto& pl : p.places) {
    mix_str(h, pl.place_id);
    mix_num(h, pl.x_m);
    mix_num(h, pl.y_m);
    mix_num(h, pl.yaw_rad);
  }
  return h;
}

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

Json package_to_json(const MapPackage& p) {
  Json::Array cells;
  cells.reserve(p.occupancy.size());
  for (int8_t c : p.occupancy) {
    cells.push_back(Json::Num(static_cast<double>(c)));
  }
  Json::Array places;
  for (const auto& pl : p.places) {
    places.push_back(Json::Obj({
        {"place_id", Json::Str(pl.place_id)},
        {"x_m", Json::Num(pl.x_m)},
        {"y_m", Json::Num(pl.y_m)},
        {"yaw_rad", Json::Num(pl.yaw_rad)},
    }));
  }
  return Json::Obj({
      {"format", Json::Str(kMapPackageFormat)},
      {"map_id", Json::Str(p.map_id)},
      {"resolution_m", Json::Num(p.resolution_m)},
      {"origin_m", Json::Arr({Json::Num(p.origin_x_m), Json::Num(p.origin_y_m)})},
      {"size", Json::Arr({Json::Num(static_cast<double>(p.width)),
                          Json::Num(static_cast<double>(p.height))})},
      {"occupancy", Json::Arr(std::move(cells))},
      {"places", Json::Arr(std::move(places))},
      {"checksum", Json::Num(static_cast<double>(package_checksum(p)))},
  });
}

bool valid_package(const Json& root, MapPackage& out, std::string& err,
                   std::uint32_t& stored_checksum) {
  if (!root.is_object()) {
    err = "map package root must be an object";
    return false;
  }
  const Json* format = root.find("format");
  const Json* id = root.find("map_id");
  const Json* res = root.find("resolution_m");
  const Json* origin = root.find("origin_m");
  const Json* size = root.find("size");
  const Json* occ = root.find("occupancy");
  const Json* places = root.find("places");
  const Json* checksum = root.find("checksum");
  if (format == nullptr || !format->is_str() ||
      format->as_str() != kMapPackageFormat) {
    err = "unsupported map package format";
    return false;
  }
  if (id == nullptr || !id->is_str() || id->as_str().empty()) {
    err = "map package needs a non-empty map_id";
    return false;
  }
  if (res == nullptr || !res->is_num() || !(res->as_num() > 0.0) ||
      !std::isfinite(res->as_num())) {
    err = "map package needs a finite positive resolution_m";
    return false;
  }
  if (origin == nullptr || !origin->is_array() ||
      origin->as_array().size() != 2) {
    err = "map package origin_m must be [x, y]";
    return false;
  }
  if (size == nullptr || !size->is_array() || size->as_array().size() != 2) {
    err = "map package size must be [width, height]";
    return false;
  }
  const double w = size->as_array()[0].is_num()
                       ? size->as_array()[0].as_num()
                       : -1.0;
  const double h = size->as_array()[1].is_num()
                       ? size->as_array()[1].as_num()
                       : -1.0;
  if (w != std::floor(w) || h != std::floor(h) || w <= 0.0 || h <= 0.0 ||
      w > 4096.0 || h > 4096.0 || w * h > 16777216.0) {
    err = "map package size must be a sane positive integer pair";
    return false;
  }
  if (occ == nullptr || !occ->is_array() ||
      occ->as_array().size() != static_cast<std::size_t>(w * h)) {
    err = "map package occupancy length must equal width*height";
    return false;
  }
  if (places == nullptr || !places->is_array()) {
    err = "map package places must be an array";
    return false;
  }
  if (checksum == nullptr || !checksum->is_num()) {
    err = "map package needs a numeric checksum";
    return false;
  }
  out.map_id = id->as_str();
  out.resolution_m = res->as_num();
  out.origin_x_m = origin->as_array()[0].is_num()
                       ? origin->as_array()[0].as_num()
                       : 0.0;
  out.origin_y_m = origin->as_array()[1].is_num()
                       ? origin->as_array()[1].as_num()
                       : 0.0;
  if (!std::isfinite(out.origin_x_m) || !std::isfinite(out.origin_y_m)) {
    err = "map package origin must be finite";
    return false;
  }
  out.width = static_cast<std::uint32_t>(w);
  out.height = static_cast<std::uint32_t>(h);
  out.occupancy.clear();
  out.occupancy.reserve(static_cast<std::size_t>(w * h));
  for (const auto& cell : occ->as_array()) {
    if (!cell.is_num() || (cell.as_num() != -1.0 && cell.as_num() != 0.0 &&
                           cell.as_num() != 100.0)) {
      err = "map package occupancy cells must be -1, 0, or 100";
      return false;
    }
    out.occupancy.push_back(static_cast<int8_t>(cell.as_num()));
  }
  out.places.clear();
  for (const auto& pl : places->as_array()) {
    if (!pl.is_object()) {
      err = "map package places must be objects";
      return false;
    }
    const Json* pid = pl.find("place_id");
    const Json* px = pl.find("x_m");
    const Json* py = pl.find("y_m");
    const Json* pyaw = pl.find("yaw_rad");
    if (pid == nullptr || !pid->is_str() || pid->as_str().empty() ||
        px == nullptr || !px->is_num() || !std::isfinite(px->as_num()) ||
        py == nullptr || !py->is_num() || !std::isfinite(py->as_num()) ||
        pyaw == nullptr || !pyaw->is_num() ||
        !std::isfinite(pyaw->as_num())) {
      err = "map package place needs id + finite x/y/yaw";
      return false;
    }
    out.places.push_back(
        {pid->as_str(), px->as_num(), py->as_num(), pyaw->as_num()});
  }
  const double cs = checksum->as_num();
  if (cs != std::floor(cs) || cs < 0.0 || cs > 4294967295.0) {
    err = "map package checksum must be a uint32";
    return false;
  }
  stored_checksum = static_cast<std::uint32_t>(cs);
  return true;
}

}  // namespace

MapPackage package_from_snapshot(const std::string& map_id,
                                 const robot::iface::MapSnapshot& snapshot) {
  MapPackage out;
  out.map_id = map_id;
  out.resolution_m = snapshot.resolution_m;
  out.origin_x_m = snapshot.origin_x_m;
  out.origin_y_m = snapshot.origin_y_m;
  out.width = snapshot.width;
  out.height = snapshot.height;
  out.occupancy = snapshot.occupancy;
  out.places = snapshot.places;
  return out;
}

bool save_map_package(const std::string& path, const MapPackage& package,
                       std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  if (package.map_id.empty()) {
    err = "cannot save a map package without map_id";
    return false;
  }
  if (package.width == 0 || package.height == 0 ||
      package.occupancy.size() !=
          static_cast<std::size_t>(package.width) * package.height) {
    err = "cannot save a map package with mismatched dimensions";
    return false;
  }
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    err = "cannot open file for writing: " + path;
    return false;
  }
  out << json_dump(package_to_json(package)) << "\n";
  out.flush();
  if (!out.good()) {
    err = "failed writing map package: " + path;
    return false;
  }
  return true;
}

bool load_map_package(const std::string& path, MapPackage& package,
                       std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = json_parse(text, &parse_error);
  if (root == nullptr) {
    err = "invalid map package: " + parse_error;
    return false;
  }
  MapPackage loaded;
  std::uint32_t stored = 0;
  if (!valid_package(*root, loaded, err, stored)) return false;
  if (package_checksum(loaded) != stored) {
    err = "map package checksum mismatch (file corrupt or edited)";
    return false;
  }
  package = std::move(loaded);
  return true;
}

}  // namespace lost_signal::sim::eval
