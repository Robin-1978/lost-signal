#include "map_serialize.hpp"

#include "content.hpp"

#include <nlohmann/json.hpp>

namespace game {
namespace {

nlohmann::ordered_json vec2(const sim::Vec2& value) {
 return {value.x, value.y};
}

nlohmann::ordered_json strings(const std::vector<std::string>& values) {
 nlohmann::ordered_json result = nlohmann::ordered_json::array();
 for (const auto& value : values) {
  result.push_back(value);
 }
 return result;
}

} // namespace

std::string serialize_map_canonical(const MapDefinition& map) {
 nlohmann::ordered_json obstacles = nlohmann::ordered_json::array();
 for (const auto& obstacle : map.obstacles) {
  nlohmann::ordered_json item;
  item["id"] = obstacle.id;
  item["type"] = "box";
  item["min"] = vec2(obstacle.min);
  item["max"] = vec2(obstacle.max);
  if (!obstacle.active_in.empty()) {
   item["active_in"] = strings(obstacle.active_in);
  }
  obstacles.push_back(std::move(item));
 }
 nlohmann::ordered_json surfaces = nlohmann::ordered_json::array();
 for (const auto& surface : map.surfaces) {
  nlohmann::ordered_json item;
  item["id"] = surface.id;
  item["type"] = "box";
  item["min"] = vec2(surface.min);
  item["max"] = vec2(surface.max);
  item["material"] = surface.material;
  item["label"] = surface.label;
  if (!surface.active_in.empty()) {
   item["active_in"] = strings(surface.active_in);
  }
  surfaces.push_back(std::move(item));
 }
 nlohmann::ordered_json markers = nlohmann::ordered_json::object();
 for (const auto& [id, pose] : map.markers) {
  markers[id] = {pose.x, pose.y, pose.theta};
 }
 nlohmann::ordered_json actors = nlohmann::ordered_json::array();
 for (const auto& actor : map.actors) {
  nlohmann::ordered_json path = nlohmann::ordered_json::array();
  for (const auto& point : actor.path) {
   path.push_back(vec2(point));
  }
  nlohmann::ordered_json item;
  item["id"] = actor.id;
  item["type"] = "patrol";
  item["path"] = std::move(path);
  item["speed_mps"] = actor.speed_mps;
  item["radius_m"] = actor.radius_m;
  item["active_in"] = strings(actor.active_in);
  actors.push_back(std::move(item));
 }
 nlohmann::ordered_json root;
 root["schema_version"] = 1;
 root["id"] = map.id;
 root["scene_3d"] = map.scene_3d;
 root["size_m"] = vec2(map.size_m);
 root["grid_resolution_m"] = map.grid_resolution_m;
 root["obstacles"] = std::move(obstacles);
 root["surfaces"] = std::move(surfaces);
 root["markers"] = std::move(markers);
 root["actors"] = std::move(actors);
 return root.dump(2, ' ', false) + "\n";
}

} // namespace game
