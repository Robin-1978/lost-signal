#include "sim/viewer/map_editor.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "sim/eval/world_io.hpp"

namespace lost_signal::sim::viewer {

void MapEditor::normalize(world::StaticBox& b) {
  if (b.x1 < b.x0) std::swap(b.x0, b.x1);
  if (b.y1 < b.y0) std::swap(b.y0, b.y1);
}

void MapEditor::normalize_rect(double& x0, double& y0, double& x1,
                               double& y1) {
  if (x1 < x0) std::swap(x0, x1);
  if (y1 < y0) std::swap(y0, y1);
}

void MapEditor::load_from(const world::World& world) {
 content_.survey_resolution_m = world.survey_resolution_m();
  content_.boxes = world.static_boxes();
  content_.places = world.places();
  content_.materials = world.materials();
  content_.zones = world.material_zones();
  content_.patches = world.terrain_patches();
  content_.slabs = world.support_slabs();
  content_.environment = world.environment();
 content_.initial_pose = world.initial_pose_prior();
  for (auto& b : content_.boxes) normalize(b);
  for (auto& p : content_.patches) {
    normalize_rect(p.x0, p.y0, p.x1, p.y1);
  }
  for (auto& z : content_.zones) {
    normalize_rect(z.x0, z.y0, z.x1, z.y1);
  }
  sel_box_ = -1;
  sel_corner_ = -1;
  sel_place_ = -1;
  sel_patch_ = -1;
  sel_zone_ = -1;
  pressing_ = false;
  dirty_ = false;
  message_.clear();
  place_seq_ = static_cast<int>(content_.places.size());
  patch_seq_ = static_cast<int>(content_.patches.size());
  zone_seq_ = static_cast<int>(content_.zones.size());
}

void MapEditor::apply(world::World& world) {
 world.set_survey_resolution_m(content_.survey_resolution_m);
  world.set_static_boxes(content_.boxes);
  world.set_places(content_.places);
  world.set_materials(content_.materials);
  world.set_material_zones(content_.zones);
  world.set_terrain_patches(content_.patches);
  world.set_support_slabs(content_.slabs);
  world.set_environment(content_.environment);
 world.set_initial_pose_prior(content_.initial_pose);
  dirty_ = false;
  message_ = "applied";
}

void MapEditor::press(double world_x, double world_y) {
  pressing_ = true;
  press_x_ = world_x;
  press_y_ = world_y;
  // Corner handles of the current selection first.
  if (sel_box_ >= 0 && sel_box_ < static_cast<int>(content_.boxes.size())) {
    const auto& b = content_.boxes[static_cast<std::size_t>(sel_box_)];
    const double cx[4] = {b.x0, b.x1, b.x1, b.x0};
    const double cy[4] = {b.y0, b.y0, b.y1, b.y1};
    for (int c = 0; c < 4; ++c) {
      if (std::hypot(world_x - cx[c], world_y - cy[c]) <= kGrabRadiusM) {
        sel_corner_ = c;
        sel_place_ = -1;
        sel_patch_ = -1;
        sel_zone_ = -1;
        return;
      }
    }
  }
  // Nearest box body.
  int best_box = -1;
  double best_d = kGrabRadiusM;
  for (std::size_t i = 0; i < content_.boxes.size(); ++i) {
    const auto& b = content_.boxes[i];
    const double cx =
        std::clamp(world_x, std::min(b.x0, b.x1), std::max(b.x0, b.x1));
    const double cy =
        std::clamp(world_y, std::min(b.y0, b.y1), std::max(b.y0, b.y1));
    const double d = std::hypot(world_x - cx, world_y - cy);
    if (d <= best_d) {
      best_d = d;
      best_box = static_cast<int>(i);
    }
  }
  if (best_box >= 0) {
    sel_box_ = best_box;
    sel_corner_ = -1;
    sel_place_ = -1;
    sel_patch_ = -1;
    sel_zone_ = -1;
    return;
  }
  // Nearest patch, then zone (rect bodies; same grab radius discipline).
  int best_patch = -1;
  for (std::size_t i = 0; i < content_.patches.size(); ++i) {
    const auto& p = content_.patches[i];
    const double cx =
        std::clamp(world_x, std::min(p.x0, p.x1), std::max(p.x0, p.x1));
    const double cy =
        std::clamp(world_y, std::min(p.y0, p.y1), std::max(p.y0, p.y1));
    if (std::hypot(world_x - cx, world_y - cy) <= kGrabRadiusM) {
      best_patch = static_cast<int>(i);
    }
  }
  if (best_patch >= 0) {
    sel_patch_ = best_patch;
    sel_box_ = -1;
    sel_corner_ = -1;
    sel_place_ = -1;
    sel_zone_ = -1;
    return;
  }
  int best_zone = -1;
  for (std::size_t i = 0; i < content_.zones.size(); ++i) {
    const auto& z = content_.zones[i];
    const double cx =
        std::clamp(world_x, std::min(z.x0, z.x1), std::max(z.x0, z.x1));
    const double cy =
        std::clamp(world_y, std::min(z.y0, z.y1), std::max(z.y0, z.y1));
    if (std::hypot(world_x - cx, world_y - cy) <= kGrabRadiusM) {
      best_zone = static_cast<int>(i);
    }
  }
  if (best_zone >= 0) {
    sel_zone_ = best_zone;
    sel_box_ = -1;
    sel_corner_ = -1;
    sel_place_ = -1;
    sel_patch_ = -1;
    return;
  }
  // Nearest place.
  int best_place = -1;
  best_d = kGrabRadiusM;
  for (std::size_t i = 0; i < content_.places.size(); ++i) {
    const double d =
        std::hypot(world_x - content_.places[i].x_m, world_y - content_.places[i].y_m);
    if (d <= best_d) {
      best_d = d;
      best_place = static_cast<int>(i);
    }
  }
  sel_box_ = -1;
  sel_corner_ = -1;
  sel_place_ = best_place;
  sel_patch_ = -1;
  sel_zone_ = -1;
}

void MapEditor::drag(double world_x, double world_y) {
  if (!pressing_) return;
  const double dx = clamp_to_map(world_x) - clamp_to_map(press_x_);
  const double dy = clamp_to_map(world_y) - clamp_to_map(press_y_);
  if (dx == 0.0 && dy == 0.0) return;
  if (sel_box_ >= 0 && sel_box_ < static_cast<int>(content_.boxes.size())) {
    auto& b = content_.boxes[static_cast<std::size_t>(sel_box_)];
    if (sel_corner_ < 0) {
      b.x0 += dx;
      b.x1 += dx;
      b.y0 += dy;
      b.y1 += dy;
    } else {
      if (sel_corner_ == 0 || sel_corner_ == 3) {
        b.x0 += dx;
      } else {
        b.x1 += dx;
      }
      if (sel_corner_ < 2) {
        b.y0 += dy;
      } else {
        b.y1 += dy;
      }
    }
    normalize(b);
    dirty_ = true;
  } else if (sel_patch_ >= 0 &&
             sel_patch_ < static_cast<int>(content_.patches.size())) {
    auto& p = content_.patches[static_cast<std::size_t>(sel_patch_)];
    p.x0 = clamp_to_map(p.x0 + dx);
    p.x1 = clamp_to_map(p.x1 + dx);
    p.y0 = clamp_to_map(p.y0 + dy);
    p.y1 = clamp_to_map(p.y1 + dy);
    normalize_rect(p.x0, p.y0, p.x1, p.y1);
    dirty_ = true;
  } else if (sel_zone_ >= 0 &&
             sel_zone_ < static_cast<int>(content_.zones.size())) {
    auto& z = content_.zones[static_cast<std::size_t>(sel_zone_)];
    z.x0 = clamp_to_map(z.x0 + dx);
    z.x1 = clamp_to_map(z.x1 + dx);
    z.y0 = clamp_to_map(z.y0 + dy);
    z.y1 = clamp_to_map(z.y1 + dy);
    normalize_rect(z.x0, z.y0, z.x1, z.y1);
    dirty_ = true;
  } else if (sel_place_ >= 0 &&
             sel_place_ < static_cast<int>(content_.places.size())) {
    auto& p = content_.places[static_cast<std::size_t>(sel_place_)];
    p.x_m = clamp_to_map(p.x_m + dx);
    p.y_m = clamp_to_map(p.y_m + dy);
    dirty_ = true;
  }
  press_x_ = world_x;
  press_y_ = world_y;
}

void MapEditor::release() { pressing_ = false; }

void MapEditor::add_box(double world_x, double world_y) {
  const double x = clamp_to_map(world_x), y = clamp_to_map(world_y);
  const double h = kDefaultBoxM * 0.5;
  content_.boxes.push_back(
      {x - h, y - h, x + h, y + h, kDefaultHeightM});
  sel_box_ = static_cast<int>(content_.boxes.size()) - 1;
  sel_corner_ = -1;
  sel_place_ = -1;
  sel_patch_ = -1;
  sel_zone_ = -1;
  dirty_ = true;
  message_ = "box added";
}

void MapEditor::add_place(double world_x, double world_y) {
  world::NamedPlace p;
  p.place_id = "edit_place_" + std::to_string(place_seq_++);
  p.x_m = clamp_to_map(world_x);
  p.y_m = clamp_to_map(world_y);
  p.yaw_rad = 0.0;
  content_.places.push_back(p);
  sel_place_ = static_cast<int>(content_.places.size()) - 1;
  sel_box_ = -1;
  sel_corner_ = -1;
  sel_patch_ = -1;
  sel_zone_ = -1;
  dirty_ = true;
  message_ = "place added: " + p.place_id;
}

void MapEditor::add_patch(double world_x, double world_y) {
  const double x = clamp_to_map(world_x), y = clamp_to_map(world_y);
  world::TerrainPatch p;
  p.patch_id = "edit_patch_" + std::to_string(patch_seq_++);
  p.x0 = x - 1.0;
  p.y0 = y - 1.0;
  p.x1 = x + 1.0;
  p.y1 = y + 1.0;
  p.base_height_m = 0.0;
  p.grade_x = 0.2;  // default ramp, main's map-editor default
  content_.patches.push_back(p);
  sel_patch_ = static_cast<int>(content_.patches.size()) - 1;
  sel_box_ = -1;
  sel_corner_ = -1;
  sel_place_ = -1;
  sel_zone_ = -1;
  dirty_ = true;
  message_ = "patch added: " + p.patch_id;
}

void MapEditor::add_zone(double world_x, double world_y,
                         const std::string& material) {
  const double x = clamp_to_map(world_x), y = clamp_to_map(world_y);
  world::MaterialZone z;
  z.zone_id = "edit_zone_" + std::to_string(zone_seq_++);
  z.x0 = x - 1.0;
  z.y0 = y - 1.0;
  z.x1 = x + 1.0;
  z.y1 = y + 1.0;
  z.material_id = material;
  content_.zones.push_back(z);
  sel_zone_ = static_cast<int>(content_.zones.size()) - 1;
  sel_box_ = -1;
  sel_corner_ = -1;
  sel_place_ = -1;
  sel_patch_ = -1;
  dirty_ = true;
  message_ = "zone added: " + z.zone_id;
}

void MapEditor::remove_selected() {
  if (sel_box_ >= 0 && sel_box_ < static_cast<int>(content_.boxes.size())) {
    content_.boxes.erase(content_.boxes.begin() + sel_box_);
    sel_box_ = -1;
    sel_corner_ = -1;
    dirty_ = true;
    message_ = "box removed";
  } else if (sel_patch_ >= 0 &&
             sel_patch_ < static_cast<int>(content_.patches.size())) {
    message_ =
        "patch removed: " +
        content_.patches[static_cast<std::size_t>(sel_patch_)].patch_id;
    content_.patches.erase(content_.patches.begin() + sel_patch_);
    sel_patch_ = -1;
    dirty_ = true;
  } else if (sel_zone_ >= 0 &&
             sel_zone_ < static_cast<int>(content_.zones.size())) {
    message_ = "zone removed: " +
               content_.zones[static_cast<std::size_t>(sel_zone_)].zone_id;
    content_.zones.erase(content_.zones.begin() + sel_zone_);
    sel_zone_ = -1;
    dirty_ = true;
  } else if (sel_place_ >= 0 &&
             sel_place_ < static_cast<int>(content_.places.size())) {
    message_ = "place removed: " + content_.places[static_cast<std::size_t>(sel_place_)].place_id;
    content_.places.erase(content_.places.begin() + sel_place_);
    sel_place_ = -1;
    dirty_ = true;
  }
}

bool MapEditor::save(const std::string& path) {
  std::string err;
  if (!eval::save_world_file(path, content_, &err)) {
    message_ = "save failed: " + err;
    return false;
  }
  // Validity proof: reload the whole file and compare every section.
  eval::WorldFileContent rc;
  if (!eval::load_world_file(path, rc, &err)) {
    message_ = "save wrote unreadable file: " + err;
    return false;
  }
  if (rc.boxes.size() != content_.boxes.size() ||
 rc.survey_resolution_m != content_.survey_resolution_m ||
      rc.places.size() != content_.places.size() ||
      rc.materials.size() != content_.materials.size() ||
      rc.zones.size() != content_.zones.size() ||
      rc.patches.size() != content_.patches.size() ||
      rc.slabs.size() != content_.slabs.size() ||
      rc.environment.size() != content_.environment.size()) {
    message_ = "save round-trip mismatch";
    return false;
  }
  dirty_ = false;
  message_ = "saved " + path;
  return true;
}

}  // namespace lost_signal::sim::viewer
