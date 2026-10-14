#include "map_editor.hpp"

#include "game/content_catalog.hpp"
#include "game/map_serialize.hpp"

#include <cmath>
#include <fstream>

namespace client {
namespace {

double distance(double ax, double ay, double bx, double by) {
 return std::hypot(ax - bx, ay - by);
}

} // namespace

MapEditor::MapEditor(const game::ContentCatalog& catalog, std::string level_id)
    : catalog_(catalog), level_id_(std::move(level_id)) {
 const auto& level = catalog_.level(level_id_);
 working_ = catalog_.map(level.map);
}

void MapEditor::press(double world_x, double world_y) {
 selected_kind_ = Kind::kNone;
 selected_id_.clear();
 double best = kGrabRadiusM;
 for (const auto& [id, pose] : working_.markers) {
  const double distance_to = distance(world_x, world_y, pose.x, pose.y);
  if (distance_to < best) {
   best = distance_to;
   selected_kind_ = Kind::kMarker;
   selected_id_ = id;
  }
 }
 for (const auto& obstacle : working_.obstacles) {
  const double center_x = (obstacle.min.x + obstacle.max.x) * 0.5;
  const double center_y = (obstacle.min.y + obstacle.max.y) * 0.5;
  const double distance_to = distance(world_x, world_y, center_x, center_y);
  if (distance_to < best) {
   best = distance_to;
   selected_kind_ = Kind::kObstacle;
   selected_id_ = obstacle.id;
  }
 }
 for (const auto& surface : working_.surfaces) {
  const double center_x = (surface.min.x + surface.max.x) * 0.5;
  const double center_y = (surface.min.y + surface.max.y) * 0.5;
  const double distance_to = distance(world_x, world_y, center_x, center_y);
  if (distance_to < best) {
   best = distance_to;
   selected_kind_ = Kind::kSurface;
   selected_id_ = surface.id;
  }
 }
 if (selected_kind_ != Kind::kNone) {
  last_x_ = world_x;
  last_y_ = world_y;
  tracking_ = true;
 }
}

void MapEditor::drag(double world_x, double world_y) {
 if (selected_kind_ == Kind::kNone || !tracking_) {
  return;
 }
 const double dx = world_x - last_x_;
 const double dy = world_y - last_y_;
 last_x_ = world_x;
 last_y_ = world_y;
 const double size_x = working_.size_m.x;
 const double size_y = working_.size_m.y;
 auto shift_box = [&](sim::Vec2& min, sim::Vec2& max) {
  const double width = max.x - min.x;
  const double height = max.y - min.y;
  min.x = std::clamp(min.x + dx, 0.0, size_x - width);
  min.y = std::clamp(min.y + dy, 0.0, size_y - height);
  max.x = min.x + width;
  max.y = min.y + height;
 };
 if (selected_kind_ == Kind::kMarker) {
  auto found = working_.markers.find(selected_id_);
  if (found != working_.markers.end()) {
   found->second.x = std::clamp(found->second.x + dx, 0.0, size_x);
   found->second.y = std::clamp(found->second.y + dy, 0.0, size_y);
   dirty_ = true;
  }
 } else if (selected_kind_ == Kind::kObstacle) {
  for (auto& obstacle : working_.obstacles) {
   if (obstacle.id == selected_id_) {
    shift_box(obstacle.min, obstacle.max);
    dirty_ = true;
   }
  }
 } else if (selected_kind_ == Kind::kSurface) {
  for (auto& surface : working_.surfaces) {
   if (surface.id == selected_id_) {
    shift_box(surface.min, surface.max);
    dirty_ = true;
   }
  }
 }
}

void MapEditor::release() {
 tracking_ = false;
 selected_kind_ = Kind::kNone;
 selected_id_.clear();
}

bool MapEditor::save(const std::string& assets_root) {
 const std::string text = game::serialize_map_canonical(working_);
 const std::string path = assets_root + "/maps/" + working_.id + ".json";
 {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
   message_ = "存盘失败：打不开 " + path;
   return false;
  }
  file << text;
 }
 try {
  // Validity proof: a full catalog reload parses + validates everything
  // (schema, references, unlock chains). Throws on any breakage.
  game::ContentCatalog::load(assets_root);
 } catch (const std::exception& error) {
  message_ = std::string("存盘已写但校验失败：") + error.what();
  return false;
 }
 dirty_ = false;
 message_ = "已存盘并通过校验：maps/" + working_.id + ".json（重进关卡生效）";
 return true;
}

} // namespace client
