#include "robot/autonomy/behavior/crossing_map_semantics.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <string_view>

namespace sim {
namespace {

bool parse_number(const runtime::SemanticFeature& feature,
 std::string_view key, double& value, bool required = true) {
 const auto found = feature.attributes.find(std::string(key));
 if (found == feature.attributes.end())
 return !required;
 errno = 0;
 char* end = nullptr;
 const char* begin = found->second.c_str();
 value = std::strtod(begin, &end);
 return errno == 0 && end != begin && *end == '\0' && std::isfinite(value);
}

CrossingMapSemantics reject(std::string diagnostic) {
 CrossingMapSemantics result;
 result.valid = false;
 result.diagnostic = std::move(diagnostic);
 return result;
}

} // namespace

CrossingMapSemantics crossing_semantics_from_map(
 const runtime::MapSnapshot& map) {
 CrossingMapSemantics result;
 std::map<std::string, std::size_t> zone_indices;

 for (const auto& feature : map.features) {
 if (feature.kind != "conflict_zone")
 continue;
 if (feature.id.empty() || zone_indices.contains(feature.id))
 return reject("duplicate or empty conflict-zone id");
 if (feature.geometry != runtime::SemanticGeometry::Polygon ||
 feature.vertices_m.size() < 3) {
 return reject("conflict zone '" + feature.id + "' must be a polygon");
 }
 Vec2 minimum{
 std::numeric_limits<double>::infinity(),
 std::numeric_limits<double>::infinity(),
 };
 Vec2 maximum{
 -std::numeric_limits<double>::infinity(),
 -std::numeric_limits<double>::infinity(),
 };
 for (const auto vertex : feature.vertices_m) {
 if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y))
 return reject("conflict zone '" + feature.id + "' contains a non-finite vertex");
 minimum.x = std::min(minimum.x, vertex.x);
 minimum.y = std::min(minimum.y, vertex.y);
 maximum.x = std::max(maximum.x, vertex.x);
 maximum.y = std::max(maximum.y, vertex.y);
 }
 if (!(maximum.x > minimum.x) || !(maximum.y > minimum.y))
 return reject("conflict zone '" + feature.id + "' has no area");

 const auto authority_it = feature.attributes.find("authority");
 if (authority_it == feature.attributes.end() ||
 (authority_it->second != "control" && authority_it->second != "shadow")) {
 return reject("conflict zone '" + feature.id + "' has invalid authority");
 }
 zone_indices.emplace(feature.id, result.zones.size());
 result.zones.push_back({
 {feature.id, minimum, maximum, {}},
 authority_it->second == "control"
 ? CrossingAuthority::Control
 : CrossingAuthority::Observe,
 });
 }

 for (const auto& feature : map.features) {
 if (feature.kind != "conflict_approach")
 continue;
 const auto owner = feature.attributes.find("conflict_zone_id");
 if (owner == feature.attributes.end() || !zone_indices.contains(owner->second))
 return reject("conflict approach '" + feature.id + "' references an unknown zone");
 if (feature.geometry != runtime::SemanticGeometry::Polyline ||
 feature.vertices_m.size() < 2) {
 return reject("conflict approach '" + feature.id + "' must be a polyline");
 }
 for (const auto vertex : feature.vertices_m) {
 if (!std::isfinite(vertex.x) || !std::isfinite(vertex.y))
 return reject("conflict approach '" + feature.id + "' contains a non-finite vertex");
 }
 double half_width_m = 0.0;
 if (!parse_number(feature, "half_width_m", half_width_m) ||
 !(half_width_m > 0.0)) {
 return reject("conflict approach '" + feature.id + "' has invalid motion metadata");
 }
 auto& zone = result.zones[zone_indices.at(owner->second)].geometry;
 zone.observation_approaches.push_back({
 feature.vertices_m, half_width_m,
 });
 }

 for (const auto& zone : result.zones) {
 if (zone.authority == CrossingAuthority::Control &&
 zone.geometry.observation_approaches.empty()) {
 return reject("controlled conflict zone '" + zone.geometry.id +
 "' requires an observable approach");
 }
 }
 return result;
}

} // namespace sim
