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

std::string MapEditor::free_id(const std::string& prefix) const {
 for (int index = 1;; ++index) {
  const std::string candidate = prefix + (index < 10 ? "0" : "") + std::to_string(index);
  bool taken = false;
  for (const auto& obstacle : working_.obstacles) {
   taken = taken || obstacle.id == candidate;
  }
  for (const auto& surface : working_.surfaces) {
   taken = taken || surface.id == candidate;
  }
  taken = taken || working_.markers.count(candidate) > 0;
  if (!taken) {
   return candidate;
  }
 }
}

namespace {
// Corner order: 0 min/min, 1 max/min, 2 max/max, 3 min/max.
void corner_of(const sim::Vec2& min, const sim::Vec2& max, int corner, double& x, double& y) {
 x = (corner == 0 || corner == 3) ? min.x : max.x;
 y = (corner == 0 || corner == 1) ? min.y : max.y;
}
} // namespace

void MapEditor::press(double world_x, double world_y) {
 // Corner handles of the current selection win over re-picking, so a
 // selected box can be resized without losing it to a neighbor.
 selected_corner_ = -1;
 if (selected_kind_ == Kind::kObstacle || selected_kind_ == Kind::kSurface) {
  sim::Vec2 min{0.0, 0.0};
  sim::Vec2 max{0.0, 0.0};
  bool found = false;
  if (selected_kind_ == Kind::kObstacle) {
   for (const auto& obstacle : working_.obstacles) {
    if (obstacle.id == selected_id_) {
     min = obstacle.min;
     max = obstacle.max;
     found = true;
    }
   }
  } else {
   for (const auto& surface : working_.surfaces) {
    if (surface.id == selected_id_) {
     min = surface.min;
     max = surface.max;
     found = true;
    }
   }
  }
  if (found) {
   for (int corner = 0; corner < 4; ++corner) {
    double cx = 0.0;
    double cy = 0.0;
    corner_of(min, max, corner, cx, cy);
    if (distance(world_x, world_y, cx, cy) < kGrabRadiusM) {
     selected_corner_ = corner;
     last_x_ = world_x;
     last_y_ = world_y;
     tracking_ = true;
     return;
    }
   }
  }
 }
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
 // Corner resize: drag one corner, opposite corner anchored, 0.2 m minimum
 // edge so the validator (positive area) can never trip on a save.
 if (selected_corner_ >= 0 && (selected_kind_ == Kind::kObstacle || selected_kind_ == Kind::kSurface)) {
  auto resize = [&](sim::Vec2& min, sim::Vec2& max) {
   if (selected_corner_ == 0 || selected_corner_ == 3) {
    min.x = std::clamp(min.x + dx, 0.0, max.x - 0.2);
   } else {
    max.x = std::clamp(max.x + dx, min.x + 0.2, size_x);
   }
   if (selected_corner_ == 0 || selected_corner_ == 1) {
    min.y = std::clamp(min.y + dy, 0.0, max.y - 0.2);
   } else {
    max.y = std::clamp(max.y + dy, min.y + 0.2, size_y);
   }
   dirty_ = true;
  };
  if (selected_kind_ == Kind::kObstacle) {
   for (auto& obstacle : working_.obstacles) {
    if (obstacle.id == selected_id_) {
     resize(obstacle.min, obstacle.max);
    }
   }
  } else {
   for (auto& surface : working_.surfaces) {
    if (surface.id == selected_id_) {
     resize(surface.min, surface.max);
    }
   }
  }
  return;
 }
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
 selected_corner_ = -1;
 selected_kind_ = Kind::kNone;
 selected_id_.clear();
}

void MapEditor::add_obstacle(double world_x, double world_y) {
 game::BoxDefinition obstacle;
 obstacle.id = free_id("edit_box_");
 obstacle.min = {world_x - 0.5, world_y - 0.5};
 obstacle.max = {world_x + 0.5, world_y + 0.5};
 obstacle.min.x = std::clamp(obstacle.min.x, 0.0, working_.size_m.x - 1.0);
 obstacle.min.y = std::clamp(obstacle.min.y, 0.0, working_.size_m.y - 1.0);
 obstacle.max = {obstacle.min.x + 1.0, obstacle.min.y + 1.0};
 obstacle.active_in = {level_id_};
 working_.obstacles.push_back(std::move(obstacle));
 selected_kind_ = Kind::kObstacle;
 selected_id_ = working_.obstacles.back().id;
 selected_corner_ = -1;
 dirty_ = true;
 message_ = catalog_.tr("editor.new_obstacle") + selected_id_ + catalog_.tr("editor.new_obstacle_local");
}

void MapEditor::add_surface(double world_x, double world_y) {
 // Numbers come from the material library, never literals: the paved default
 // is behavior-neutral by content, not by a C++ constant.
 const auto& paved = catalog_.surface_material("paved");
 game::SurfaceDefinition surface;
 surface.id = free_id("edit_surface_");
 surface.min = {world_x - 1.0, world_y - 1.0};
 surface.max = {world_x + 1.0, world_y + 1.0};
 surface.min.x = std::clamp(surface.min.x, 0.0, working_.size_m.x - 2.0);
 surface.min.y = std::clamp(surface.min.y, 0.0, working_.size_m.y - 2.0);
 surface.max = {surface.min.x + 2.0, surface.min.y + 2.0};
 surface.material = paved.id;
 surface.active_in = {level_id_};
 surface.label = catalog_.tr("editor.new_ground_default");
 surface.traction_scale = paved.traction_scale;
 surface.friction_mu = paved.friction_mu;
 surface.rolling_drag = paved.rolling_drag;
 working_.surfaces.push_back(std::move(surface));
 selected_kind_ = Kind::kSurface;
 selected_id_ = working_.surfaces.back().id;
 selected_corner_ = -1;
 dirty_ = true;
 message_ = catalog_.tr("editor.new_ground") + selected_id_ + catalog_.tr("editor.new_ground_paved");
}

void MapEditor::add_marker(double world_x, double world_y) {
 const std::string id = free_id("edit_marker_");
 working_.markers.emplace(id, game::Pose{world_x, world_y, 0.0});
 auto& pose = working_.markers.at(id);
 pose.x = std::clamp(pose.x, 0.0, working_.size_m.x);
 pose.y = std::clamp(pose.y, 0.0, working_.size_m.y);
 selected_kind_ = Kind::kMarker;
 selected_id_ = id;
 selected_corner_ = -1;
 dirty_ = true;
 message_ = catalog_.tr("editor.new_marker") + selected_id_;
}

void MapEditor::remove_selected() {
 if (selected_kind_ == Kind::kObstacle) {
  for (auto it = working_.obstacles.begin(); it != working_.obstacles.end(); ++it) {
   if (it->id == selected_id_) {
    message_ = catalog_.tr("editor.delete_obstacle") + selected_id_;
    working_.obstacles.erase(it);
    break;
   }
  }
 } else if (selected_kind_ == Kind::kSurface) {
  for (auto it = working_.surfaces.begin(); it != working_.surfaces.end(); ++it) {
   if (it->id == selected_id_) {
    message_ = catalog_.tr("editor.delete_ground") + selected_id_;
    working_.surfaces.erase(it);
    break;
   }
  }
 } else if (selected_kind_ == Kind::kMarker) {
  message_ = catalog_.tr("editor.delete_marker") + selected_id_;
  working_.markers.erase(selected_id_);
 } else {
  return;
 }
 dirty_ = true;
 selected_kind_ = Kind::kNone;
 selected_id_.clear();
 selected_corner_ = -1;
}

bool MapEditor::save(const std::string& assets_root) {
 const std::string text = game::serialize_map_canonical(working_);
 const std::string path = assets_root + "/maps/" + working_.id + ".json";
 {
  std::ofstream file(path, std::ios::binary | std::ios::trunc);
  if (!file) {
   message_ = catalog_.tr("editor.save_fail") + path;
   return false;
  }
  file << text;
 }
 try {
  // Validity proof: a full catalog reload parses + validates everything
  // (schema, references, unlock chains). Throws on any breakage.
  game::ContentCatalog::load(assets_root);
 } catch (const std::exception& error) {
  message_ = catalog_.tr("editor.save_invalid") + error.what();
  return false;
 }
 dirty_ = false;
 message_ = catalog_.trf("editor.saved", {working_.id});
 return true;
}

} // namespace client
