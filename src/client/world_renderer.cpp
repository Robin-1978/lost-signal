#include "world_renderer.hpp"

#include "scene_renderer_3d.hpp"
#include "game/content.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace client {
namespace {

constexpr Color kVoid{8, 10, 14, 255};
constexpr Color kFloorDark{29, 31, 31, 255};
constexpr Color kFloorLight{42, 43, 40, 255};
constexpr Color kFloorJoint{72, 74, 68, 72};
constexpr Color kCyan{73, 226, 203, 255};
constexpr Color kRoute{103, 205, 194, 255};
constexpr Color kAmber{245, 171, 72, 255};
constexpr Color kDanger{241, 93, 89, 255};
constexpr Color kViolet{158, 124, 232, 255};
constexpr Color kText{239, 238, 229, 255};
constexpr Color kMuted{151, 157, 153, 255};
constexpr double kRadToDeg = 57.29577951308232;

float clampf(float value, float low, float high) {
 return std::max(low, std::min(value, high));
}

Color mix(Color from, Color to, float amount, unsigned char alpha = 255) {
 const float t = clampf(amount, 0.0f, 1.0f);
 const auto channel = [t](unsigned char a, unsigned char b) {
 return static_cast<unsigned char>(static_cast<float>(a) + (static_cast<float>(b) - a) * t);
 };
 return {channel(from.r, to.r), channel(from.g, to.g), channel(from.b, to.b), alpha};
}

Color safety_color(double value, unsigned char alpha = 176) {
 const float t = clampf(static_cast<float>(value), 0.0f, 1.0f);
 if (t < 0.58f) {
 return mix(Color{54, 185, 141, 255}, kAmber, t / 0.58f, alpha);
 }
 return mix(kAmber, kDanger, (t - 0.58f) / 0.42f, alpha);
}

double route_heading_at(const std::vector<sim::Vec2>& path, sim::Vec2 point, double fallback) {
 double best_distance_sq = std::numeric_limits<double>::infinity();
 double best_heading = fallback;
 for (std::size_t index = 1; index < path.size(); ++index) {
 const double dx = path[index].x - path[index - 1].x;
 const double dy = path[index].y - path[index - 1].y;
 const double length_sq = dx * dx + dy * dy;
 if (length_sq < 1e-12)
 continue;
 const double projection = std::clamp(
 ((point.x - path[index - 1].x) * dx + (point.y - path[index - 1].y) * dy) / length_sq, 0.0,
 1.0);
 const double nearest_x = path[index - 1].x + dx * projection;
 const double nearest_y = path[index - 1].y + dy * projection;
 const double distance_sq = (point.x - nearest_x) * (point.x - nearest_x) +
 (point.y - nearest_y) * (point.y - nearest_y);
 if (distance_sq < best_distance_sq) {
 best_distance_sq = distance_sq;
 best_heading = std::atan2(dy, dx);
 }
 }
 return best_heading;
}

std::string fixed(double value, int precision = 1) {
 std::ostringstream output;
 output << std::fixed << std::setprecision(precision) << value;
 return output.str();
}

float distance(Vector2 a, Vector2 b) {
 const float dx = a.x - b.x;
 const float dy = a.y - b.y;
 return std::sqrt(dx * dx + dy * dy);
}

Vector2 lerp(Vector2 a, Vector2 b, float amount) {
 return {a.x + (b.x - a.x) * amount, a.y + (b.y - a.y) * amount};
}

float hash01(int x, int y, int salt = 0) {
 unsigned value = static_cast<unsigned>(x) * 374761393U + static_cast<unsigned>(y) * 668265263U +
                  static_cast<unsigned>(salt) * 1442695041U;
 value = (value ^ (value >> 13U)) * 1274126177U;
 return static_cast<float>((value ^ (value >> 16U)) & 0xffffU) / 65535.0f;
}

Vector2 polar(Vector2 center, float radius, float angle) {
 return {center.x + std::cos(angle) * radius, center.y - std::sin(angle) * radius};
}

void draw_soft_disc(Vector2 center, float radius, Color color) {
 DrawCircleGradient(center, radius, Fade(color, 0.14f), Fade(color, 0.0f));
}

} // namespace

std::filesystem::path find_ui_font(const std::filesystem::path& assets) {
 const std::filesystem::path candidates[] = {
   assets / "fonts/NotoSansCJKsc-Regular.otf",
   "assets/fonts/NotoSansCJKsc-Regular.otf",
   "C:/Windows/Fonts/msyh.ttc",
   "/System/Library/Fonts/PingFang.ttc",
 };
 for (const auto& path : candidates) {
  if (!path.empty() && std::filesystem::is_regular_file(path))
   return path;
 }
 return {};
}

WorldRenderer::WorldRenderer(const std::filesystem::path& font_path, const std::filesystem::path& assets_root,
                             const game::ContentCatalog& catalog)
    : catalog_(catalog) {
 if (!font_path.empty()) {
  std::string glyph_text = "失落的信号第一章醒来洛一机库配置自主执行中任务完成"
                           "安全迅速节能时间速度路程能耗净空规划感知预测等待叉车"
                           "补给站信号塔机器人地图雷达定位路径运动回放暂停继续"
                           "区域协议恢复链路锁定冲突警戒湿滑货架任务进度当前位置"
                           "现场画面地面电量消耗行动通过前往叉车路线米秒我先等它"
                           "上层航向轮速编码器闭环目标实测局部轨迹动态图候选"
                           "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
                           "：，。·//.-+()%";
  std::error_code error;
  for (std::filesystem::recursive_directory_iterator
         iterator(assets_root, std::filesystem::directory_options::skip_permission_denied, error),
       end;
       iterator != end; iterator.increment(error)) {
   if (error) {
    error.clear();
    continue;
   }
   if (!iterator->is_regular_file() || iterator->path().extension() != ".json")
    continue;
   std::ifstream input(iterator->path(), std::ios::binary);
   glyph_text.append(std::istreambuf_iterator<char>(input), {});
  }
  int codepoint_count = 0;
  int* decoded = LoadCodepoints(glyph_text.c_str(), &codepoint_count);
  std::vector<int> codepoints;
  if (decoded && codepoint_count > 0)
   codepoints.assign(decoded, decoded + codepoint_count);
  UnloadCodepoints(decoded);
  std::sort(codepoints.begin(), codepoints.end());
  codepoints.erase(std::unique(codepoints.begin(), codepoints.end()), codepoints.end());
  font_ = LoadFontEx(font_path.string().c_str(), 48, codepoints.data(), static_cast<int>(codepoints.size()));
  owns_font_ = font_.texture.id != 0;
 }
 if (!owns_font_)
  font_ = GetFontDefault();
 for (const auto& [id, material] : catalog_.surface_materials()) {
  (void)id;
  if (!material.texture_albedo.empty() && !ground_textures_.contains(material.texture_albedo)) {
   Texture2D tex = LoadTexture((assets_root / material.texture_albedo).string().c_str());
   if (tex.id > 0) {
    SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(tex, TEXTURE_WRAP_REPEAT);
    ground_textures_.emplace(material.texture_albedo, tex);
   }
  }
 }
 scene_3d_ = std::make_unique<SceneRenderer3D>(assets_root, catalog);
 {
  // Engine sprite (not content): failure here only loses puffs, never blocks.
  Texture2D puff = LoadTexture((assets_root / "textures/environment/dust_puff.png").string().c_str());
  if (puff.id > 0) {
   SetTextureFilter(puff, TEXTURE_FILTER_BILINEAR);
   dust_sprite_ = puff;
  }
 }
}

WorldRenderer::~WorldRenderer() {
 if (owns_font_)
  UnloadFont(font_);
 if (dust_sprite_.id > 0)
  UnloadTexture(dust_sprite_);
}

std::optional<sim::Vec2> WorldRenderer::screen_to_world(Vector2 screen) const {
 if (!drawn_once_) {
  return std::nullopt;
 }
 if (!analysis_mode_) {
 return scene_3d_->screen_to_ground(screen);
 }
 if (scale_ <= 0.0f)
 return std::nullopt;
 return sim::Vec2{(screen.x - world_origin_.x) / scale_, (world_origin_.y - screen.y) / scale_};
}

std::optional<std::string> WorldRenderer::garage_slot_at(Vector2 screen) const {
 for (const auto& hotspot : garage_hotspots_) {
 if (CheckCollisionPointCircle(screen, hotspot.center, hotspot.radius) ||
 CheckCollisionPointRec(screen, hotspot.label)) {
 return hotspot.slot;
 }
 }
 return std::nullopt;
}

void WorldRenderer::draw_edit_overlay(const game::MissionSimulation& mission, const MapEditor& editor) const {
 if (!editor.active()) {
  return;
 }
 const auto& working = editor.working();
 const Color ghost = Fade(kAmber, 0.55f);
 const Color hot = Fade(kAmber, 0.95f);
 if (!analysis_mode_) {
 const auto height_at = [](const game::TerrainPatchDefinition& patch, double x, double y) {
 return patch.base_height_m + patch.grade_x * (x - patch.min.x) +
 patch.grade_y * (y - patch.min.y);
 };
 BeginScissorMode(static_cast<int>(viewport_.x), static_cast<int>(viewport_.y),
 static_cast<int>(viewport_.width), static_cast<int>(viewport_.height));
 BeginMode3D(scene_3d_->camera());
 for (const auto& obstacle : working.obstacles) {
 const bool selected = editor.selected_kind() == MapEditor::Kind::kObstacle &&
 editor.selected_id() == obstacle.id;
 const Vector3 center{static_cast<float>((obstacle.min.x + obstacle.max.x) * 0.5), 0.75f,
 static_cast<float>(-(obstacle.min.y + obstacle.max.y) * 0.5)};
 const Vector3 size{static_cast<float>(obstacle.max.x - obstacle.min.x), 1.5f,
 static_cast<float>(obstacle.max.y - obstacle.min.y)};
 DrawCubeV(center, size, Fade(selected ? hot : ghost, 0.18f));
 DrawCubeWiresV(center, size, selected ? hot : ghost);
 }
 for (const auto& surface : working.surfaces) {
 const bool selected = editor.selected_kind() == MapEditor::Kind::kSurface &&
 editor.selected_id() == surface.id;
 const Vector3 center{static_cast<float>((surface.min.x + surface.max.x) * 0.5), 0.025f,
 static_cast<float>(-(surface.min.y + surface.max.y) * 0.5)};
 const Vector3 size{static_cast<float>(surface.max.x - surface.min.x), 0.05f,
 static_cast<float>(surface.max.y - surface.min.y)};
 DrawCubeV(center, size, Fade(selected ? hot : kCyan, 0.20f));
 DrawCubeWiresV(center, size, selected ? hot : Fade(kCyan, 0.65f));
 }
 for (const auto& patch : working.terrain) {
 const bool selected = editor.selected_kind() == MapEditor::Kind::kTerrain &&
 editor.selected_id() == patch.id;
 const Color tint = selected ? hot : Color{158, 124, 232, 190};
 const Vector3 a{static_cast<float>(patch.min.x),
 static_cast<float>(height_at(patch, patch.min.x, patch.min.y) + 0.025),
 static_cast<float>(-patch.min.y)};
 const Vector3 b{static_cast<float>(patch.max.x),
 static_cast<float>(height_at(patch, patch.max.x, patch.min.y) + 0.025),
 static_cast<float>(-patch.min.y)};
 const Vector3 c{static_cast<float>(patch.max.x),
 static_cast<float>(height_at(patch, patch.max.x, patch.max.y) + 0.025),
 static_cast<float>(-patch.max.y)};
 const Vector3 d{static_cast<float>(patch.min.x),
 static_cast<float>(height_at(patch, patch.min.x, patch.max.y) + 0.025),
 static_cast<float>(-patch.max.y)};
 DrawTriangle3D(a, c, b, Fade(tint, 0.23f));
 DrawTriangle3D(a, d, c, Fade(tint, 0.23f));
 DrawLine3D(a, b, tint);
 DrawLine3D(b, c, tint);
 DrawLine3D(c, d, tint);
 DrawLine3D(d, a, tint);
 DrawLine3D(a, c, Fade(tint, 0.7f));
 }
 for (const auto& [id, pose] : working.markers) {
 const bool selected = editor.selected_kind() == MapEditor::Kind::kMarker && editor.selected_id() == id;
 DrawCylinderWires({static_cast<float>(pose.x), 0.02f, static_cast<float>(-pose.y)},
 0.22f, 0.22f, 0.12f, 20, selected ? hot : ghost);
 }
 EndMode3D();
 EndScissorMode();
 const std::string status =
 (editor.dirty() ? catalog_.tr("editor.dirty_on") : catalog_.tr("editor.dirty_off")) +
 editor.message() + catalog_.tr("editor.help");
 draw_text(status, {viewport_.x + 18, viewport_.y + viewport_.height - 24}, 16, Fade(kText, 0.9f));
 return;
 }
 (void)mission;
 auto rect_lines = [&](sim::Vec2 min, sim::Vec2 max, Color color) {
  const auto top_left = world_to_screen({min.x, max.y});
  const auto bottom_right = world_to_screen({max.x, min.y});
  DrawRectangleLinesEx({top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y},
                       2.0f, color);
 };
 for (const auto& obstacle : working.obstacles) {
  const bool selected =
    editor.selected_kind() == MapEditor::Kind::kObstacle && editor.selected_id() == obstacle.id;
  rect_lines(obstacle.min, obstacle.max, selected ? hot : ghost);
  if (selected) {
   for (int corner = 0; corner < 4; ++corner) {
    const auto px = world_to_screen({corner == 0 || corner == 3 ? obstacle.min.x : obstacle.max.x,
                                     corner == 0 || corner == 1 ? obstacle.min.y : obstacle.max.y});
    DrawRectangleLinesEx({px.x - 5, px.y - 5, 10, 10}, 2.0f, hot);
   }
  }
 }
 for (const auto& surface : working.surfaces) {
  const bool selected =
    editor.selected_kind() == MapEditor::Kind::kSurface && editor.selected_id() == surface.id;
  rect_lines(surface.min, surface.max, selected ? hot : Fade(kCyan, 0.45f));
  if (selected) {
   for (int corner = 0; corner < 4; ++corner) {
    const auto px = world_to_screen({corner == 0 || corner == 3 ? surface.min.x : surface.max.x,
                                     corner == 0 || corner == 1 ? surface.min.y : surface.max.y});
    DrawRectangleLinesEx({px.x - 5, px.y - 5, 10, 10}, 2.0f, hot);
   }
  }
 }
 for (const auto& terrain : working.terrain) {
 const bool selected =
 editor.selected_kind() == MapEditor::Kind::kTerrain && editor.selected_id() == terrain.id;
 rect_lines(terrain.min, terrain.max, selected ? hot : Fade(kViolet, 0.62f));
 if (selected) {
 for (int corner = 0; corner < 4; ++corner) {
 const auto px = world_to_screen({corner == 0 || corner == 3 ? terrain.min.x : terrain.max.x,
 corner == 0 || corner == 1 ? terrain.min.y : terrain.max.y});
 DrawRectangleLinesEx({px.x - 5, px.y - 5, 10, 10}, 2.0f, hot);
 }
 const auto center = world_to_screen({(terrain.min.x + terrain.max.x) * 0.5,
 (terrain.min.y + terrain.max.y) * 0.5});
 draw_text(terrain.id + " h=" + fixed(terrain.base_height_m, 2) + "m slope=" +
 fixed(std::atan(std::hypot(terrain.grade_x, terrain.grade_y)) * kRadToDeg, 1) + "°",
 {center.x + 10, center.y - 10}, 14, hot);
 }
 }
 for (const auto& [id, pose] : working.markers) {
  const bool selected = editor.selected_kind() == MapEditor::Kind::kMarker && editor.selected_id() == id;
  const auto center = world_to_screen({pose.x, pose.y});
  DrawCircleLines(static_cast<int>(center.x), static_cast<int>(center.y), 9.0f, selected ? hot : ghost);
  if (selected) {
   draw_text(id, {center.x + 12, center.y - 8}, 15, hot);
  }
 }
 const std::string status =
   (editor.dirty() ? catalog_.tr("editor.dirty_on") : catalog_.tr("editor.dirty_off")) + editor.message() +
   catalog_.tr("editor.help");
 draw_text(status, {viewport_.x + 18, viewport_.y + viewport_.height - 24}, 16, Fade(kText, 0.9f));
}

Vector2 WorldRenderer::world_to_screen(sim::Vec2 point) const {
 return {
   world_origin_.x + static_cast<float>(point.x) * scale_,
   world_origin_.y - static_cast<float>(point.y) * scale_,
 };
}

void WorldRenderer::draw_text(const std::string& text, Vector2 position, float size, Color color) const {
 DrawTextEx(font_, text.c_str(), position, size, 0.25f, color);
}

void WorldRenderer::draw(const game::MissionSimulation& mission, int screen_width, int screen_height,
 const std::string& chapter_title, bool assembly_view) {
 ClearBackground(kVoid);
 DrawRectangleGradientV(0, 0, screen_width, screen_height, {24, 25, 24, 255}, kVoid);

 drawn_once_ = true;
 const float available_width = std::max(320.0f, static_cast<float>(screen_width) - 360.0f);
 viewport_ = {18.0f, 76.0f, available_width - 36.0f,
              std::max(360.0f, static_cast<float>(screen_height) - 104.0f)};

 if (!analysis_mode_) {
  scene_3d_->draw(mission, screen_width, screen_height,
 {0.0f, 0.0f, available_width, static_cast<float>(screen_height)}, assembly_view);
  draw_header(mission, static_cast<int>(available_width), chapter_title);
  draw_telemetry(mission, screen_height);
  draw_view_mode();
 if (assembly_view)
 draw_garage_overlay(mission);
 else
 garage_hotspots_.clear();
  return;
 }

 garage_hotspots_.clear();

 const auto map_size = mission.map_definition().size_m;
 const bool active = mission.snapshot().status == game::MissionStatus::Running;
 if (active) {
  scale_ = clampf(std::min(viewport_.width / 12.2f, viewport_.height / 8.3f), 50.0f, 92.0f);
  sim::Vec2 focus{mission.snapshot().robot.x, mission.snapshot().robot.y};
 if (mission.endless_mode()) {
 focus.x = mission.snapshot().robot.x * 0.52 + mission.field_operator().position.x * 0.48;
 focus.y = mission.snapshot().robot.y * 0.52 + mission.field_operator().position.y * 0.48;
 }
  if (!mission.snapshot().dynamic_obstacles.empty()) {
   const auto& detected = mission.snapshot().dynamic_obstacles.front();
   focus_hold_.x = mission.snapshot().robot.x * 0.58 + detected.estimated_position.x * 0.42;
   focus_hold_.y = mission.snapshot().robot.y * 0.58 + detected.estimated_position.y * 0.42 + 0.80;
   // Hysteresis on track birth/death (display only): hold the last threat
   // blend a second past loss so the frame doesn't snap on flicker.
   focus_hold_until_ = mission.snapshot().time_s + 1.0;
  }
  if (mission.snapshot().time_s <= focus_hold_until_) {
   focus = focus_hold_;
 } else if (!mission.endless_mode()) {
   focus.x += 1.2 * std::cos(mission.snapshot().robot.theta);
   focus.y += 1.2 * std::sin(mission.snapshot().robot.theta);
  }
  // Ease the camera (display only, sim untouched): 5Hz estimate steps no
  // longer teleport the frame; fast pans still catch up within ~6 frames.
  if (!focus_init_) {
   focus_smooth_ = focus;
   focus_init_ = true;
  } else {
   focus_smooth_.x += (focus.x - focus_smooth_.x) * 0.35;
   focus_smooth_.y += (focus.y - focus_smooth_.y) * 0.35;
  }
  focus = focus_smooth_;
  const double visible_width = viewport_.width / scale_;
  const double visible_height = viewport_.height / scale_;
  const double camera_x =
    std::clamp(focus.x - visible_width * 0.5, 0.0, std::max(0.0, map_size.x - visible_width));
  const double camera_y =
    std::clamp(focus.y - visible_height * 0.5, 0.0, std::max(0.0, map_size.y - visible_height));
  world_origin_ = {
    viewport_.x - static_cast<float>(camera_x) * scale_,
    viewport_.y + viewport_.height + static_cast<float>(camera_y) * scale_,
  };
 } else {
  scale_ = std::max(4.0f, std::min(viewport_.width / static_cast<float>(map_size.x),
                                   viewport_.height / static_cast<float>(map_size.y)));
  const float map_width = static_cast<float>(map_size.x) * scale_;
  const float map_height = static_cast<float>(map_size.y) * scale_;
  world_origin_ = {
    viewport_.x + (viewport_.width - map_width) * 0.5f,
    viewport_.y + (viewport_.height + map_height) * 0.5f,
  };
 }

 draw_header(mission, static_cast<int>(available_width), chapter_title);
 draw_world(mission);
 draw_telemetry(mission, screen_height);
 draw_view_mode();
}

void WorldRenderer::draw_garage_overlay(const game::MissionSimulation& mission) {
 garage_hotspots_.clear();
 const auto& robot = mission.robot();
 const auto& presentation = catalog_.presentation_for_level(mission.level().id);
 const auto& state = mission.snapshot();
 const double cosine = std::cos(state.robot.theta);
 const double sine = std::sin(state.robot.theta);
 const float ground_height =
 static_cast<float>(state.robot_height_m - robot.collision_height_m * 0.5);
 const Vector2 mouse = GetMousePosition();

 std::size_t visible_index = 0;
 for (const auto& column : presentation.component_columns) {
 const auto slot = std::find_if(robot.slots.begin(), robot.slots.end(),
 [&](const auto& candidate) { return candidate.slot == column.slot; });
 if (slot == robot.slots.end())
 continue;
 game::Vec3 local{};
 std::size_t mount_count = 0;
 for (const auto& mount_id : slot->mounts) {
 const auto mount = std::find_if(robot.mounts.begin(), robot.mounts.end(),
 [&](const auto& candidate) { return candidate.id == mount_id; });
 if (mount == robot.mounts.end())
 continue;
 local.x += mount->position_m.x;
 local.y += mount->position_m.y;
 local.z += mount->position_m.z;
 ++mount_count;
 }
 if (mount_count > 0) {
 local.x /= static_cast<double>(mount_count);
 local.y /= static_cast<double>(mount_count);
 local.z /= static_cast<double>(mount_count);
 } else {
 // Motor controllers live inside the chassis and intentionally have no
 // exposed physical mount. Give the body slot a stable service hatch.
 local = {0.0, 0.0, 0.16};
 }
 const double mount_x = state.robot.x + cosine * local.x - sine * local.y;
 const double mount_y = state.robot.y + sine * local.x + cosine * local.y;
 const Vector3 world_mount{static_cast<float>(mount_x), ground_height + static_cast<float>(local.z + 0.05),
 static_cast<float>(-mount_y)};
 const Vector2 anchor = GetWorldToScreenEx(world_mount, scene_3d_->camera(),
 static_cast<int>(viewport_.x + viewport_.width + 18.0f), GetScreenHeight());

 const bool left = visible_index % 2 == 0;
 const std::size_t row = visible_index / 2;
 const float card_width = 176.0f;
 const float card_height = 54.0f;
 const float card_x = left ? viewport_.x + 22.0f
 : viewport_.x + viewport_.width - card_width - 22.0f;
 const float card_y = viewport_.y + 82.0f + static_cast<float>(row) * 92.0f;
 const Rectangle card{card_x, card_y, card_width, card_height};
 const bool hovered = CheckCollisionPointCircle(mouse, anchor, 28.0f) ||
 CheckCollisionPointRec(mouse, card);
 const Color accent = hovered ? kAmber : kCyan;
 const Vector2 joint{left ? card.x + card.width : card.x, card.y + card.height * 0.5f};
 DrawLineEx(anchor, joint, hovered ? 2.5f : 1.5f, Fade(accent, hovered ? 0.90f : 0.55f));
 DrawCircleV(anchor, hovered ? 14.0f : 11.0f, Fade(accent, 0.16f));
 DrawCircleLines(static_cast<int>(anchor.x), static_cast<int>(anchor.y), hovered ? 15.0f : 12.0f,
 accent);
 DrawCircleV(anchor, 3.5f, accent);
 DrawRectangleRounded(card, 0.18f, 8, hovered ? Color{35, 49, 46, 244} : Color{15, 21, 21, 226});
 DrawRectangleRoundedLinesEx(card, 0.18f, 8, hovered ? 2.0f : 1.0f, Fade(accent, 0.78f));
 draw_text(column.title, {card.x + 12.0f, card.y + 8.0f}, 15, accent);
 const auto& selected = mission.build().part(column.slot);
 const std::string installed = selected.empty() ? catalog_.tr("garage.mount_empty")
 : catalog_.component(selected).title;
 draw_text(installed, {card.x + 12.0f, card.y + 29.0f}, 14, Fade(kText, 0.86f));
 garage_hotspots_.push_back({column.slot, anchor, card, 28.0f});
 ++visible_index;
 }

 const std::string hint = catalog_.tr("garage.assembly_hint");
 const auto measured = MeasureTextEx(font_, hint.c_str(), 16, 0.25f);
 const Rectangle pill{viewport_.x + (viewport_.width - measured.x - 30.0f) * 0.5f,
 viewport_.y + viewport_.height - 50.0f, measured.x + 30.0f, 32.0f};
 DrawRectangleRounded(pill, 0.45f, 10, {12, 18, 18, 224});
 draw_text(hint, {pill.x + 15.0f, pill.y + 7.0f}, 16, Fade(kText, 0.88f));
}

void WorldRenderer::draw_view_mode() const {
 const std::string text =
   analysis_mode_ ? catalog_.tr("world.view_cognitive") : catalog_.tr("world.view_explore");
 const auto measured = MeasureTextEx(font_, text.c_str(), 16, 0.25f);
 const Rectangle pill{21.0f, 70.0f, measured.x + 26.0f, 29.0f};
 DrawRectangleRounded(pill, 0.46f, 10, analysis_mode_ ? Color{24, 61, 64, 235} : Color{16, 20, 19, 205});
 draw_text(text, {pill.x + 13.0f, pill.y + 6.0f}, 16, analysis_mode_ ? kCyan : Fade(kText, 0.82f));
}

void WorldRenderer::draw_header(const game::MissionSimulation& mission, int width,
                                const std::string& chapter_title) const {
 draw_text(chapter_title, {22, 14}, 16, Fade(kMuted, 0.88f));
 draw_text(mission.level().title, {21, 35}, 26, kText);

 const auto status = mission.snapshot().status;
 std::string text = catalog_.tr("world.status_deploy");
 Color color = kAmber;
 if (status == game::MissionStatus::Running) {
 text = mission.endless_mode()
 ? catalog_.tr("world.mode_field")
 : (mission.level().control_mode == "manual_calibration"
 ? catalog_.tr("world.mode_manual")
 : catalog_.tr("world.mode_auto"));
  color = kCyan;
 } else if (status == game::MissionStatus::Succeeded) {
  text = catalog_.tr("world.restored");
  color = kCyan;
 } else if (status == game::MissionStatus::Failed) {
  text = catalog_.tr("world.interrupted");
  color = kDanger;
 }
 const auto measured = MeasureTextEx(font_, text.c_str(), 14, 0.25f);
 const Rectangle badge{static_cast<float>(width) - measured.x - 43, 23, measured.x + 25, 31};
 DrawRectangleRounded(badge, 0.45f, 12, {18, 21, 21, 218});
 DrawCircleV({badge.x + 13, badge.y + 15.5f}, 4.0f, color);
 draw_text(text, {badge.x + 23, badge.y + 7}, 17, kText);
}

void WorldRenderer::draw_world(const game::MissionSimulation& mission) const {
 DrawRectangleRounded({viewport_.x - 5, viewport_.y - 5, viewport_.width + 10, viewport_.height + 10}, 0.012f,
                      8, {13, 15, 15, 255});
 DrawRectangleGradientV(static_cast<int>(viewport_.x), static_cast<int>(viewport_.y),
                        static_cast<int>(viewport_.width), static_cast<int>(viewport_.height), kFloorLight,
                        kFloorDark);

 BeginScissorMode(static_cast<int>(viewport_.x), static_cast<int>(viewport_.y),
                  static_cast<int>(viewport_.width), static_cast<int>(viewport_.height));
 const auto& map = mission.map_definition();

 // Large concrete slabs, sparse wear and soft light create a place instead of a graph-paper grid.
 for (int x = 0; x < static_cast<int>(map.size_m.x); x += 3) {
  for (int y = 0; y < static_cast<int>(map.size_m.y); y += 3) {
   const auto top_left = world_to_screen({static_cast<double>(x), static_cast<double>(y + 3)});
   const auto bottom_right = world_to_screen({static_cast<double>(x + 3), static_cast<double>(y)});
   const Rectangle tile{top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y};
   const float variation = hash01(x, y);
   const auto shade = static_cast<unsigned char>(variation * 8.0f);
   DrawRectangleRec(tile,
                    Color{static_cast<unsigned char>(30 + shade), static_cast<unsigned char>(32 + shade),
                          static_cast<unsigned char>(30 + shade), 96});
   DrawLineEx({tile.x, tile.y + tile.height}, {tile.x + tile.width, tile.y + tile.height}, 1.0f, kFloorJoint);
   DrawLineEx({tile.x + tile.width, tile.y}, {tile.x + tile.width, tile.y + tile.height}, 1.0f, kFloorJoint);
   if (variation > 0.58f) {
    const Vector2 stain{tile.x + tile.width * (0.2f + hash01(x, y, 2) * 0.6f),
                        tile.y + tile.height * (0.2f + hash01(x, y, 3) * 0.6f)};
    DrawEllipse(static_cast<int>(stain.x), static_cast<int>(stain.y), 10.0f + variation * 17.0f,
                4.0f + variation * 7.0f, {14, 16, 15, 24});
   }
  }
 }

 // Local pools of light keep the eye on characters and destinations.
 draw_soft_disc(world_to_screen({mission.snapshot().robot.x, mission.snapshot().robot.y}), scale_ * 1.75f,
                {83, 173, 157, 255});
 for (const auto& obstacle : mission.snapshot().dynamic_obstacles) {
  draw_soft_disc(world_to_screen(obstacle.estimated_position), scale_ * 2.1f, Color{170, 78, 59, 255});
 }
 if (mission.endless_mode()) {
 draw_soft_disc(world_to_screen(mission.field_operator().position), scale_ * 1.15f,
 {235, 169, 71, 255});
 const auto& dispatch = mission.field_dispatch();
 if (dispatch.active)
 draw_soft_disc(world_to_screen(dispatch.target), scale_ * 1.55f, {224, 166, 70, 255});
 const auto& incident = mission.world_activity().incident;
 if (incident.active)
 draw_soft_disc(world_to_screen(incident.position), scale_ * 1.75f, {224, 82, 70, 255});
 } else if (const auto* active = mission.active_goal()) {
 const auto goal = map.markers.find(active->marker);
 if (goal != map.markers.end())
 draw_soft_disc(world_to_screen({goal->second.x, goal->second.y}), scale_ * 1.55f,
 active->action == "complete" ? Color{63, 159, 148, 255} : Color{224, 166, 70, 255});
 }

 const auto map_top_left = world_to_screen({0, map.size_m.y});
 const auto map_bottom_right = world_to_screen({map.size_m.x, 0});
 DrawRectangleLinesEx(
   {map_top_left.x, map_top_left.y, map_bottom_right.x - map_top_left.x, map_bottom_right.y - map_top_left.y},
   5, {67, 65, 57, 210});
 for (const auto& surface : map.surfaces) {
  if (!game::active_in_level(surface.active_in, mission.level().id))
   continue;
  const auto top_left = world_to_screen({surface.min.x, surface.max.y});
  const auto bottom_right = world_to_screen({surface.max.x, surface.min.y});
  Rectangle box{top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y};
  // Material albedo tiled under the procedural ellipses: the ground reads as
  // its physics (sand looks sandy). Missing texture keeps the old look.
  const auto& material = catalog_.surface_material(surface.material);
  const auto tile = ground_textures_.find(material.texture_albedo);
  if (!material.texture_albedo.empty() && tile != ground_textures_.end()) {
   // Manual tiling (raylib 6 has no DrawTextureTiled): 256px source tiles at
   // world-meter scale so ground reads at true size, not stretched.
   const float tile_world_m = 256.0f / 64.0f;
   const float tile_px = tile_world_m * scale_;
   const Rectangle source{0.0f, 0.0f, static_cast<float>(tile->second.width),
                          static_cast<float>(tile->second.height)};
   for (float ty = box.y; ty < box.y + box.height; ty += tile_px) {
    for (float tx = box.x; tx < box.x + box.width; tx += tile_px) {
     const float w = std::min(tile_px, box.x + box.width - tx);
     const float h = std::min(tile_px, box.y + box.height - ty);
     const Rectangle src{0.0f, 0.0f, source.width * w / tile_px, source.height * h / tile_px};
     DrawTexturePro(tile->second, src, {tx, ty, w, h}, {0.0f, 0.0f}, 0.0f, {255, 255, 255, 110});
    }
   }
  }
  for (int index = 0; index < 7; ++index) {
   const float px = box.x + box.width * (0.10f + hash01(index, 7, 1) * 0.80f);
   const float py = box.y + box.height * (0.15f + hash01(index, 9, 2) * 0.70f);
   const float rx = box.width * (0.10f + hash01(index, 5, 3) * 0.18f);
   const float ry = box.height * (0.06f + hash01(index, 3, 4) * 0.12f);
   DrawEllipse(static_cast<int>(px + 4), static_cast<int>(py + 6), rx, ry, {9, 14, 15, 65});
   DrawEllipse(static_cast<int>(px), static_cast<int>(py), rx, ry, {37, 101, 103, 72});
   DrawLineEx({px - rx * 0.45f, py - ry * 0.32f}, {px + rx * 0.25f, py - ry * 0.55f}, 1.4f,
              {156, 211, 194, 70});
  }
  DrawRectangleRounded({box.x + 10, box.y + 10, 87, 24}, 0.45f, 8, {20, 29, 28, 195});
  DrawCircleV({box.x + 23, box.y + 22}, 4, {104, 198, 184, 220});
  draw_text(surface.label, {box.x + 34, box.y + 15}, 15, Fade(kText, 0.78f));
 }

 draw_terrain_layer(mission);

 for (const auto& actor : map.actors) {
  if (!game::active_in_level(actor.active_in, mission.level().id) || actor.path.size() < 2)
   continue;
  if (actor.kind != "patrol") {
   // Looping behaviors (wander, dwell): closed dotted polyline, no lane
   // paint (people walk, carts keep lanes). Info, not furniture.
   for (std::size_t i = 0; i < actor.path.size(); ++i) {
    const auto a = world_to_screen(actor.path[i]);
    const auto b = world_to_screen(actor.path[(i + 1) % actor.path.size()]);
    DrawLineEx(a, b, 1.5f, {23, 160, 150, 90});
   }
   continue;
  }
  const auto start = world_to_screen(actor.path.front());
  const auto end = world_to_screen(actor.path.back());
  const auto middle = lerp(start, end, 0.5f);
  DrawLineEx({start.x, start.y - 38}, {end.x, end.y - 38}, 2.0f, {198, 161, 91, 42});
  DrawLineEx({start.x, start.y + 38}, {end.x, end.y + 38}, 2.0f, {198, 161, 91, 42});
  // Direction chevrons: flat paint on the lane (info, not furniture), one per
  // meter, pointing along patrol travel. Static geometry — phase comes from
  // the moving forklift itself.
  {
   const float dx = end.x - start.x;
   const float dy = end.y - start.y;
   const float len = std::max(1.0f, std::hypot(dx, dy));
   const float ux = dx / len;
   const float uy = dy / len;
   const float step = scale_;
   for (float d = step * 0.5f; d < len; d += step) {
    const float cx = start.x + ux * d;
    const float cy = start.y + uy * d;
    const float s = 7.0f;
    DrawLineEx({cx - ux * s - uy * s * 0.6f, cy - uy * s - ux * s * 0.6f}, {cx, cy}, 2.0f,
               {198, 161, 91, 70});
    DrawLineEx({cx, cy}, {cx - ux * s + uy * s * 0.6f, cy - uy * s + ux * s * 0.6f}, 2.0f,
               {198, 161, 91, 70});
   }
  }
  draw_text(catalog_.tr("world.forklift_lane"), {middle.x - 35, middle.y + 52}, 17, {198, 181, 137, 62});
 }
 const auto& path = mission.planned_path();
 std::size_t route_start = 0;
 double route_distance = 1.0e12;
 for (std::size_t index = 0; index < path.size(); ++index) {
  const double dx = path[index].x - mission.snapshot().robot.x;
  const double dy = path[index].y - mission.snapshot().robot.y;
  const double candidate = dx * dx + dy * dy;
  if (candidate < route_distance) {
   route_distance = candidate;
   route_start = index;
  }
 }
 for (std::size_t index = std::max<std::size_t>(1, route_start + 1); index < path.size(); ++index) {
  const auto from = world_to_screen(path[index - 1]);
  const auto to = world_to_screen(path[index]);
  DrawLineEx(from, to, 6.0f, {4, 10, 10, 38});
  DrawLineEx(from, to, 2.2f, Fade(kRoute, 0.42f));
  if ((index - route_start) % 8 == 0) {
   const auto point = lerp(from, to, 0.5f);
   const float angle = std::atan2(to.y - from.y, to.x - from.x);
   const auto nose = polar(point, 7.0f, -angle);
   const auto left = polar(point, 5.0f, -angle + 2.42f);
   const auto right = polar(point, 5.0f, -angle - 2.42f);
   DrawTriangle(nose, left, right, Fade(kRoute, 0.78f));
  }
 }
 const auto& history = mission.replay();
 const std::size_t history_begin = history.size() > 1600 ? history.size() - 1600 : 0;
 for (std::size_t index = history_begin + 2; index < history.size(); index += 2) {
  const auto& before = history[index - 2].robot;
  const auto& after = history[index].robot;
  const auto from = world_to_screen({before.x, before.y});
  const auto to = world_to_screen({after.x, after.y});
  DrawLineEx(from, to, 4.0f, {5, 7, 7, 80});
  DrawLineEx(from, to, 1.6f, Fade(kText, 0.38f));
 }

 // DWA is a local, short-horizon planner. Keep its sampled fan visually
 // subordinate to the A* route, then emphasize the selected candidate.
 const auto& control = mission.snapshot().control;
 if (control.dwa_active) {
  for (const auto& candidate : control.dwa_candidates) {
   const Color color = candidate.collision ? Fade(kDanger, 0.17f) : Fade(kViolet, 0.16f);
   for (std::size_t index = 1; index < candidate.points.size(); ++index) {
    DrawLineEx(world_to_screen(candidate.points[index - 1]), world_to_screen(candidate.points[index]), 1.3f,
               color);
   }
  }
  for (std::size_t index = 1; index < control.dwa_selected.size(); ++index) {
   const auto from = world_to_screen(control.dwa_selected[index - 1]);
   const auto to = world_to_screen(control.dwa_selected[index]);
   DrawLineEx(from, to, 5.0f, {7, 9, 9, 110});
   DrawLineEx(from, to, 2.4f, Fade(kAmber, 0.92f));
  }
  const auto local_target = world_to_screen(control.local_target);
  DrawRing(local_target, 8.0f, 10.0f, 0, 360, 28, Fade(kAmber, 0.82f));
  DrawLineEx({local_target.x - 13, local_target.y}, {local_target.x + 13, local_target.y}, 1.5f,
             Fade(kAmber, 0.62f));
  DrawLineEx({local_target.x, local_target.y - 13}, {local_target.x, local_target.y + 13}, 1.5f,
             Fade(kAmber, 0.62f));
 }

 for (const auto& obstacle : map.obstacles) {
  if (!game::active_in_level(obstacle.active_in, mission.level().id))
   continue;
  const auto top_left = world_to_screen({obstacle.min.x, obstacle.max.y});
  const auto bottom_right = world_to_screen({obstacle.max.x, obstacle.min.y});
  Rectangle box{top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y};
  DrawRectangleRounded({box.x + 10, box.y + 13, box.width + 3, box.height + 3}, 0.025f, 5, {6, 7, 7, 145});
  DrawRectangleGradientH(static_cast<int>(box.x), static_cast<int>(box.y), static_cast<int>(box.width),
                         static_cast<int>(box.height), {61, 58, 50, 255}, {39, 42, 40, 255});
  DrawRectangleLinesEx(box, 2.0f, {100, 96, 80, 220});
  const bool vertical = box.height >= box.width;
  const float length = vertical ? box.height : box.width;
  const int bays = std::max(1, static_cast<int>(length / 58.0f));
  for (int bay = 0; bay < bays; ++bay) {
   const float start = length * static_cast<float>(bay) / bays;
   const float end = length * static_cast<float>(bay + 1) / bays;
   if (vertical) {
    DrawLineEx({box.x + 2, box.y + end}, {box.x + box.width - 2, box.y + end}, 2.0f, {19, 21, 20, 205});
    const Color cargo = bay % 3 == 0   ? Color{123, 78, 48, 255}
                        : bay % 3 == 1 ? Color{61, 83, 76, 255}
                                       : Color{91, 88, 67, 255};
    DrawRectangleRounded(
      {box.x + 9, box.y + start + 8, std::max(6.0f, box.width - 18), std::max(8.0f, end - start - 15)}, 0.08f,
      3, cargo);
    DrawLineEx({box.x + 13, box.y + start + 14}, {box.x + box.width - 13, box.y + start + 14}, 1,
               Fade(kText, 0.16f));
   } else {
    DrawLineEx({box.x + end, box.y + 2}, {box.x + end, box.y + box.height - 2}, 2.0f, {19, 21, 20, 205});
   }
  }
  const float cap_y[] = {box.y + 8, box.y + box.height - 13};
  for (float y : cap_y) {
   DrawRectangle(static_cast<int>(box.x - 3), static_cast<int>(y), 6, 10, {29, 30, 27, 255});
   DrawRectangle(static_cast<int>(box.x + box.width - 3), static_cast<int>(y), 6, 10, {29, 30, 27, 255});
  }
  DrawLineEx({box.x + 2, box.y + 3}, {box.x + box.width - 2, box.y + 3}, 3.0f, {142, 132, 105, 150});
  DrawLineEx({box.x + 3, box.y + 4}, {box.x + 3, box.y + box.height - 4}, 3.0f, {118, 108, 87, 120});
  DrawCircleV({box.x + 6, box.y + 7}, 2.0f, {193, 174, 128, 150});
  DrawCircleV({box.x + box.width - 6, box.y + 7}, 2.0f, {193, 174, 128, 150});
 }

 const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(mission.snapshot().time_s * 4.0));
 const auto draw_marker_at = [&](sim::Vec2 world_position, Color color, const std::string& label,
 bool tower) {
 const auto position = world_to_screen(world_position);
  draw_soft_disc(position, 46 + pulse * 7, color);
  DrawCircleV({position.x + 5, position.y + 8}, 21, {7, 8, 8, 135});
  DrawCircleV(position, 18, tower ? Color{62, 66, 62, 255} : Color{83, 72, 54, 255});
  DrawRing(position, 15, 18, 0, 360, 48, {111, 108, 91, 255});
  if (tower) {
   DrawCircleV(position, 8, {23, 31, 30, 255});
   DrawCircleV(position, 4.5f + pulse * 1.5f, color);
   DrawRing(position, 25 + pulse * 5, 26 + pulse * 5, 0, 360, 64, Fade(color, 0.28f));
  } else {
   // Via points are info, not furniture: the robot MUST drive through this
   // spot (via_radius), so no box may be drawn here — flat ring + label only
   // (same honesty language as the arrival circle).
   DrawRing(position, 15, 18, 0, 360, 48, {111, 108, 91, 255});
   DrawRing(position, 24 + pulse * 4, 25 + pulse * 4, 0, 360, 48, Fade(color, 0.35f));
  }
  const auto measured = MeasureTextEx(font_, label.c_str(), 13, 0.25f);
  const Rectangle tag{position.x - measured.x * 0.5f - 9, position.y + 27, measured.x + 18, 24};
  DrawRectangleRounded(tag, 0.42f, 8, {17, 20, 19, 215});
  draw_text(label, {tag.x + 9, tag.y + 5}, 16, kText);
 };
 const auto draw_marker = [&](const std::string& id, Color color, const std::string& label, bool tower) {
 const auto found = map.markers.find(id);
 if (found != map.markers.end())
 draw_marker_at({found->second.x, found->second.y}, color, label, tower);
 };
 if (mission.endless_mode()) {
 const auto& incident = mission.world_activity().incident;
 if (incident.active) {
 std::string label;
 switch (incident.kind) {
 case game::WorldIncidentKind::RoadClosure:
 label = catalog_.tr("incident.road_closure");
 break;
 case game::WorldIncidentKind::FallenCargo:
 label = catalog_.tr("incident.fallen_cargo");
 break;
 case game::WorldIncidentKind::PowerFault:
 label = catalog_.tr("incident.power_fault");
 break;
 }
 draw_marker_at(incident.position, kDanger, label, false);
 }
 const auto& intent = mission.field_intent();
 if (intent.active && intent.kind != game::FieldIntentKind::Follow &&
 intent.kind != game::FieldIntentKind::Wait) {
 const Color command_color = intent.acknowledgement == game::FieldIntentAcknowledgement::Rejected
 ? kDanger
 : (intent.kind == game::FieldIntentKind::Search ? kAmber : kCyan);
 std::string label;
 switch (intent.kind) {
 case game::FieldIntentKind::GoTo:
 label = catalog_.tr("field.intent.go_to");
 break;
 case game::FieldIntentKind::Search:
 label = catalog_.tr("field.intent.search");
 break;
 case game::FieldIntentKind::Return:
 label = catalog_.tr("field.intent.return");
 break;
 case game::FieldIntentKind::Follow:
 case game::FieldIntentKind::Wait:
 break;
 }
 draw_marker_at(intent.target, command_color, label,
 intent.kind == game::FieldIntentKind::Search);
 }
 const auto& dispatch = mission.field_dispatch();
 if (dispatch.active) {
 std::string label;
 if (dispatch.kind == "delivery") {
 label = dispatch.stage == 0 ? catalog_.tr("field.marker.pickup")
 : catalog_.tr("field.marker.delivery");
 } else if (dispatch.kind == "survey") {
 label = catalog_.tr("field.marker.survey");
 } else {
 label = catalog_.tr("field.marker.assist");
 }
 draw_marker_at(dispatch.target, kAmber, label, true);
 }
 } else {
 if (const auto* active = mission.active_goal()) {
 draw_marker(active->marker, active->action == "complete" ? kCyan : kAmber,
 active->label, active->action == "complete");
 }
 }

 const auto robot_position = world_to_screen({
   mission.snapshot().robot.x,
   mission.snapshot().robot.y,
 });
 std::size_t ray_index = 0;
 (void)ray_index;
 // Scan-phase sweep (display only): our lidar is a body-fixed full-ring
 // snapshot (solid-state style, not a rotating mirror), so the fan itself
 // never rotates. This faint hand marks WHEN scans fire, nothing more.
 if (!mission.build().part("sensor").empty()) {
  const double sweep_hz =
    game::property(catalog_.component(mission.build().part("sensor")).behavior, "scan_hz");
  const double phase = std::fmod(mission.snapshot().time_s * sweep_hz, 1.0) * 2.0 * sim::kPi;
  const Vector2 hand{robot_position.x + static_cast<float>(std::cos(phase)) * 70.0f,
                     robot_position.y - static_cast<float>(std::sin(phase)) * 70.0f};
  DrawLineEx(robot_position, hand, 2, {82, 222, 199, 46});
 }
 for (const auto& hit : mission.snapshot().lidar_hits) {
  // Every simulated ray is drawn (an old stride drew 1-in-4 and the scan
  // looked starved). Same data, no decimation: lines faint, hits glowing.
  const auto point = world_to_screen(hit);
  DrawLineEx(robot_position, point, 1, {82, 222, 199, 30});
  DrawCircleV(point, 2.5f, {124, 236, 211, 60});
  DrawCircleV(point, 1.5f, {150, 245, 225, 190});
 }

 // Display extrapolation shared by vehicle and pedestrian markers: propagate
 // the track by its own velocity to frame time (sim untouched). Clamped to
 // one scan period — never sail on a dead track's old velocity.
 const double scan_hz =
   mission.build().part("sensor").empty()
     ? 5.0
     : game::property(catalog_.component(mission.build().part("sensor")).behavior, "scan_hz");
 const auto shown_at = [&](const game::DynamicObstacleSnapshot& obstacle) {
  const double stale = std::clamp(mission.snapshot().time_s - obstacle.last_observed_s, 0.0, 1.0 / scan_hz);
  return sim::Vec2{obstacle.estimated_position.x + obstacle.estimated_velocity.x * stale,
                   obstacle.estimated_position.y + obstacle.estimated_velocity.y * stale};
 };

 // Tracking/risk graphics stay shared, while content's visual class chooses
 // the glyph. A pedestrian must remain recognizable as a person even though
 // LiDAR tracks are anonymous and use the same prediction pipeline.
 for (const auto& obstacle : mission.snapshot().dynamic_obstacles) {
  // Display extrapolation (see shown_at above): the track state goes stale
  // between scans (0.2s at 5Hz reads as teleporting). Propagating by its own
  // velocity to frame time is the optimal display estimate, sim untouched.
  const sim::Vec2 shown = shown_at(obstacle);
  const auto estimated = world_to_screen(shown);
  const auto position = estimated;
 const bool pedestrian = obstacle.visual_kind == "pedestrian";
 const float radius = pedestrian
 ? clampf(static_cast<float>(obstacle.radius_m) * scale_, 14.0f, 25.0f)
 : clampf(static_cast<float>(obstacle.radius_m) * scale_, 25.0f, 42.0f);
  const Color alert = kDanger;
  const auto future = world_to_screen({
    shown.x + obstacle.estimated_velocity.x * 2.5,
    shown.y + obstacle.estimated_velocity.y * 2.5,
  });
  DrawLineEx(estimated, future, 8, Fade(alert, 0.08f));
  DrawLineEx(estimated, future, 2, Fade(alert, 0.45f));
  DrawCircleV(future, radius * 0.78f, Fade(alert, 0.08f));
  DrawRing(future, radius * 0.76f, radius * 0.81f, 0, 360, 40, Fade(alert, 0.32f));
  DrawRing(estimated, radius * 0.96f, radius, 0, 360, 40, Fade(kCyan, 0.72f));
  draw_soft_disc(position, radius + 22 + pulse * 4, alert);
 const float actor_angle = static_cast<float>(-obstacle.heading_rad * 180.0 / sim::kPi);
  const float actor_radians = -actor_angle * static_cast<float>(sim::kPi) / 180.0f;
 if (pedestrian) {
 const Color clothes{75, 181, 169, 255};
 const auto head = polar(position, radius * 0.38f, actor_radians);
 const auto shoulder = polar(position, radius * 0.08f, actor_radians);
 const auto left_hand = polar(shoulder, radius * 0.42f, actor_radians + 1.5708f);
 const auto right_hand = polar(shoulder, radius * 0.42f, actor_radians - 1.5708f);
 const auto hips = polar(position, radius * 0.34f, actor_radians + static_cast<float>(sim::kPi));
 const auto left_foot = polar(hips, radius * 0.48f, actor_radians + 2.45f);
 const auto right_foot = polar(hips, radius * 0.48f, actor_radians - 2.45f);
 DrawCircleV({position.x + 4.0f, position.y + 6.0f}, radius * 0.42f, {4, 5, 5, 150});
 DrawLineEx(head, hips, std::max(5.0f, radius * 0.34f), clothes);
 DrawLineEx(left_hand, right_hand, std::max(3.0f, radius * 0.16f), {136, 218, 200, 255});
 DrawLineEx(hips, left_foot, std::max(3.0f, radius * 0.15f), {49, 62, 60, 255});
 DrawLineEx(hips, right_foot, std::max(3.0f, radius * 0.15f), {49, 62, 60, 255});
 DrawCircleV(head, radius * 0.23f, {224, 185, 151, 255});
 } else {
 const auto front = polar(position, radius * 0.94f, actor_radians);
 const auto fork_left = polar(front, radius * 0.37f, actor_radians + 1.5708f);
 const auto fork_right = polar(front, radius * 0.37f, actor_radians - 1.5708f);
 const auto fork_left_end = polar(fork_left, radius * 0.82f, actor_radians);
 const auto fork_right_end = polar(fork_right, radius * 0.82f, actor_radians);
 DrawLineEx({position.x + 7, position.y + 10}, {front.x + 7, front.y + 10}, radius * 1.35f,
 {4, 5, 5, 135});
 DrawRectanglePro({position.x + 6, position.y + 8, radius * 1.75f, radius * 1.22f},
 {radius * 0.88f, radius * 0.61f}, actor_angle, {5, 6, 6, 165});
 DrawRectanglePro({position.x, position.y, radius * 1.70f, radius * 1.16f},
 {radius * 0.85f, radius * 0.58f}, actor_angle, {217, 139, 42, 255});
 DrawRectanglePro({position.x, position.y, radius * 0.76f, radius * 0.86f},
 {radius * 0.38f, radius * 0.43f}, actor_angle, {50, 59, 58, 255});
 DrawRectanglePro({position.x, position.y, radius * 0.49f, radius * 0.60f},
 {radius * 0.245f, radius * 0.30f}, actor_angle, {107, 139, 135, 220});
 DrawLineEx(fork_left, fork_left_end, std::max(3.0f, radius * 0.16f), {83, 82, 72, 255});
 DrawLineEx(fork_right, fork_right_end, std::max(3.0f, radius * 0.16f), {83, 82, 72, 255});
 DrawCircleV(polar(position, radius * 0.50f, actor_radians + 1.5708f), radius * 0.13f,
 {24, 25, 23, 255});
 DrawCircleV(polar(position, radius * 0.50f, actor_radians - 1.5708f), radius * 0.13f,
 {24, 25, 23, 255});
 DrawCircleV(position, 3.0f + pulse * 1.4f, {255, 211, 106, 255});
 }
  const float separation =
    distance(estimated, robot_position) / scale_ - static_cast<float>(obstacle.radius_m) - 0.38f;
  const auto middle = lerp(robot_position, estimated, 0.52f);
  DrawLineEx(robot_position, estimated, 2.0f, Fade(kDanger, 0.27f));
  DrawCircleV(middle, 5, kDanger);
  const std::string warning = catalog_.trf("world.warn_conflict", {fixed(separation, 1)});
  const auto measured = MeasureTextEx(font_, warning.c_str(), 16, 0.25f);
  const Rectangle tag{estimated.x + radius + 8, estimated.y - radius - 10, measured.x + 18, 27};
  DrawRectangleRounded(tag, 0.38f, 8, {53, 24, 24, 232});
  draw_text(warning, {tag.x + 9, tag.y + 6}, 16, {255, 191, 178, 255});
 }

 draw_dust(mission);
 draw_field_operator(mission);
 draw_robot(mission);
 EndScissorMode();
 draw_terrain_layer_panel(mission);

 const float card_width = 328.0f;
 const Rectangle card{
   viewport_.x + viewport_.width - card_width - 18.0f,
   viewport_.y + 17.0f,
   card_width,
   174.0f,
 };
 DrawRectangleRounded(card, 0.08f, 10, {14, 18, 19, 235});
 DrawRectangleLinesEx(card, 1.0f, Fade(control.dwa_active ? kAmber : kCyan, 0.34f));
 draw_text(catalog_.tr("world.diag_title"), {card.x + 15, card.y + 12}, 18, kText);
 const std::string planner =
   control.dwa_active ? catalog_.trf("world.dwa_active", {std::to_string(control.dwa_candidates.size())})
                      : catalog_.tr("world.dwa_idle");
 draw_text(planner, {card.x + 15, card.y + 36}, 15, control.dwa_active ? kAmber : Fade(kCyan, 0.82f));
 draw_text(catalog_.trf("world.heading", {fixed(control.heading_error_rad, 2), fixed(control.target_v_mps, 2),
                                          fixed(control.target_w_radps, 2)}),
           {card.x + 15, card.y + 58}, 15, Fade(kText, 0.82f));
 const bool has_steering_servo = control.steering_servo_present;
 draw_text(has_steering_servo ? catalog_.tr("world.wheels_ack") : catalog_.tr("world.wheels_diff"),
           {card.x + 15, card.y + 80}, 14, Fade(kMuted, 0.82f));
 draw_text(
   catalog_.trf("world.wheel_l", {fixed(control.left_target_rpm, 0), fixed(control.left_measured_rpm, 0),
                                  fixed(control.left_pwm * 100.0, 0)}),
   {card.x + 15, card.y + 99}, 15, Fade(kText, 0.86f));
 if (has_steering_servo) {
  draw_text(
    catalog_.trf("world.wheel_servo", {fixed(control.steer_target_rad, 2), fixed(control.steer_angle_rad, 2),
                                       fixed(control.steer_pwm * 100.0, 0)}),
    {card.x + 15, card.y + 130}, 15, Fade(kText, 0.86f));
 } else {
  draw_text(
    catalog_.trf("world.wheel_r", {fixed(control.right_target_rpm, 0), fixed(control.right_measured_rpm, 0),
                                   fixed(control.right_pwm * 100.0, 0)}),
    {card.x + 15, card.y + 130}, 15, Fade(kText, 0.86f));
 }
 const auto draw_pwm = [&](float y, double value) {
  const Rectangle track{card.x + 15, y, card.width - 30, 7};
  DrawRectangleRounded(track, 0.5f, 6, {38, 43, 42, 230});
  const float center = track.x + track.width * 0.5f;
  const float extent = static_cast<float>(std::clamp(value, -1.0, 1.0)) * track.width * 0.5f;
  const Rectangle fill{extent >= 0.0f ? center : center + extent, y, std::abs(extent), track.height};
  DrawRectangleRounded(fill, 0.5f, 6, value >= 0.0 ? kCyan : kAmber);
  DrawLine(static_cast<int>(center), static_cast<int>(y - 1), static_cast<int>(center),
           static_cast<int>(y + track.height + 1), Fade(kText, 0.45f));
 };
 draw_pwm(card.y + 117, control.left_pwm);
 draw_pwm(card.y + 148, has_steering_servo ? control.steer_pwm : control.right_pwm);
}

void WorldRenderer::draw_field_operator(const game::MissionSimulation& mission) const {
 if (!mission.endless_mode())
 return;
 const auto& field_operator = mission.field_operator();
 const auto center = world_to_screen(field_operator.position);
 const float radius = clampf(scale_ * 0.25f, 15.0f, 24.0f);
 const float heading = static_cast<float>(field_operator.heading_rad);
 const float stride = field_operator.moving
 ? std::sin(static_cast<float>(mission.snapshot().time_s * 11.0)) * radius * 0.28f
 : 0.0f;
 const Color jacket{230, 151, 57, 255};
 const Color trousers{48, 64, 66, 255};

 DrawCircleV({center.x + 4.0f, center.y + 6.0f}, radius * 0.72f, {4, 5, 5, 150});
 draw_soft_disc(center, radius + 15.0f, kAmber);
 const auto head = polar(center, radius * 0.52f, heading);
 const auto hips = polar(center, radius * 0.28f, heading + static_cast<float>(sim::kPi));
 const auto left_hand = polar(center, radius * 0.72f, heading + 1.5708f);
 const auto right_hand = polar(center, radius * 0.72f, heading - 1.5708f);
 const auto left_foot = polar(hips, radius * 0.66f + stride, heading + 2.36f);
 const auto right_foot = polar(hips, radius * 0.66f - stride, heading - 2.36f);
 DrawLineEx(head, hips, std::max(6.0f, radius * 0.42f), jacket);
 DrawLineEx(left_hand, right_hand, std::max(3.0f, radius * 0.20f), jacket);
 DrawLineEx(hips, left_foot, std::max(4.0f, radius * 0.20f), trousers);
 DrawLineEx(hips, right_foot, std::max(4.0f, radius * 0.20f), trousers);
 DrawCircleV(head, radius * 0.30f, {225, 185, 148, 255});
 DrawRing(head, radius * 0.26f, radius * 0.35f, 180, 360, 18, {242, 201, 92, 255});
 DrawRing(center, radius * 0.88f, radius * 0.96f, 0, 360, 36, Fade(kAmber, 0.82f));

 const auto& intent = mission.field_intent();
 if (intent.active && intent.kind == game::FieldIntentKind::Follow) {
 const auto robot = world_to_screen({mission.snapshot().robot.x, mission.snapshot().robot.y});
 for (int segment = 0; segment < 12; segment += 2) {
 DrawLineEx(lerp(robot, center, static_cast<float>(segment) / 12.0f),
 lerp(robot, center, static_cast<float>(segment + 1) / 12.0f), 2.0f,
 Fade(kCyan, 0.72f));
 }
 }
}

void WorldRenderer::draw_terrain_layer(const game::MissionSimulation& mission) const {
 const auto& grid = mission.navigation_map();
 const auto& terrain = mission.terrain_model();
 const auto& path = mission.planned_path();
 const double resolution = grid.res();
 double minimum_height = std::numeric_limits<double>::infinity();
 double maximum_height = -std::numeric_limits<double>::infinity();
 for (int gy = 0; gy < grid.h(); ++gy) {
 for (int gx = 0; gx < grid.w(); ++gx) {
 const double height = terrain.sample(grid.gridToWorld(gx, gy)).height_m;
 minimum_height = std::min(minimum_height, height);
 maximum_height = std::max(maximum_height, height);
 }
 }
 if (!std::isfinite(minimum_height)) {
 minimum_height = 0.0;
 maximum_height = 0.0;
 }

 const auto& robot = mission.robot();
 const double support_width = mission.chassis_caps().geometry.support_width_m;
 const double total_mass = robot.mass_kg + mission.level().payload_mass_kg;
 const double center_height =
 std::max(0.05, (robot.mass_kg * robot.collision_height_m * 0.5 +
 mission.level().payload_mass_kg * mission.level().payload_cog_height_m) /
 std::max(0.5, total_mass));
 const double preview_speed = std::max(
 0.6, std::max(std::abs(mission.snapshot().robot.v), std::abs(mission.snapshot().control.target_v_mps)));

 for (int gy = 0; gy < grid.h(); ++gy) {
 for (int gx = 0; gx < grid.w(); ++gx) {
 const sim::Vec2 center = grid.gridToWorld(gx, gy);
 const auto sample = terrain.sample(center);
 const double heading = route_heading_at(path, center, mission.snapshot().robot.theta);
 Color fill{36, 45, 48, 150};
 bool blocked = !grid.isFreeCell(gx, gy);

 switch (terrain_layer_) {
 case TerrainLayer::Elevation: {
 const double range = std::max(0.05, maximum_height - minimum_height);
 const float height_t = static_cast<float>((sample.height_m - minimum_height) / range);
 fill = height_t < 0.68f
 ? mix(Color{31, 55, 89, 255}, Color{71, 204, 191, 255}, height_t / 0.68f, 172)
 : mix(Color{71, 204, 191, 255}, Color{247, 188, 92, 255}, (height_t - 0.68f) / 0.32f, 172);
 break;
 }
 case TerrainLayer::Slope: {
 const double slope_degrees = std::atan(std::hypot(sample.grade_x, sample.grade_y)) * kRadToDeg;
 fill = safety_color(slope_degrees / 35.0, 176);
 break;
 }
 case TerrainLayer::VehicleRisk: {
 const double cross_grade = sim::lateral_grade(sample, heading);
 const double bump_accel =
 sim::roughness_lateral_acceleration(sample.roughness, preview_speed, support_width);
 const double signed_bump = cross_grade < 0.0 ? -bump_accel : bump_accel;
 const double risk =
 sim::dynamic_rollover_risk(cross_grade, signed_bump, support_width, center_height);
 fill = safety_color(risk, 182);
 blocked = blocked || risk >= 1.0;
 break;
 }
 case TerrainLayer::FinalCost: {
 const sim::Vec2 next{center.x + std::cos(heading) * resolution,
 center.y + std::sin(heading) * resolution};
 const double cost = mission.navigation_traversal_cost(center, next);
 blocked = blocked || !grid.isFree(next.x, next.y) || !std::isfinite(cost);
 fill = blocked ? Color{192, 48, 53, 190} : safety_color((cost - 1.0) / 1.7, 178);
 break;
 }
 }
 if (blocked)
 fill = Color{190, 48, 53, 190};

 const auto top_left = world_to_screen({center.x - resolution * 0.5, center.y + resolution * 0.5});
 const auto bottom_right = world_to_screen({center.x + resolution * 0.5, center.y - resolution * 0.5});
 const Rectangle cell{top_left.x, top_left.y, bottom_right.x - top_left.x,
 bottom_right.y - top_left.y};
 DrawRectangleRec(cell, fill);
 DrawRectangleLinesEx(cell, 0.45f, {9, 15, 17, 42});

 if (terrain_layer_ == TerrainLayer::Slope && (gx % 3 == 1) && (gy % 3 == 1)) {
 const double magnitude = std::hypot(sample.grade_x, sample.grade_y);
 if (magnitude > 0.01) {
 const sim::Vec2 downhill{-sample.grade_x / magnitude, -sample.grade_y / magnitude};
 const auto start = world_to_screen({center.x - downhill.x * 0.12, center.y - downhill.y * 0.12});
 const auto end = world_to_screen({center.x + downhill.x * 0.17, center.y + downhill.y * 0.17});
 DrawLineEx(start, end, 2.1f, Fade(kText, 0.82f));
 const Vector2 direction{end.x - start.x, end.y - start.y};
 const float length = std::max(1.0f, std::hypot(direction.x, direction.y));
 const Vector2 unit{direction.x / length, direction.y / length};
 const Vector2 side{-unit.y, unit.x};
 DrawTriangle(end, {end.x - unit.x * 7.0f + side.x * 4.0f, end.y - unit.y * 7.0f + side.y * 4.0f},
 {end.x - unit.x * 7.0f - side.x * 4.0f, end.y - unit.y * 7.0f - side.y * 4.0f},
 Fade(kText, 0.82f));
 }
 }
 if (terrain_layer_ == TerrainLayer::VehicleRisk && (gx % 4 == 2) && (gy % 4 == 2)) {
 const sim::Vec2 direction{std::cos(heading), std::sin(heading)};
 DrawLineEx(world_to_screen({center.x - direction.x * 0.13, center.y - direction.y * 0.13}),
 world_to_screen({center.x + direction.x * 0.13, center.y + direction.y * 0.13}), 1.8f,
 Fade(kText, 0.68f));
 }
 }
 }

 // Height contours make the ramp readable even when its color changes only
 // gradually. Red edge segments in the final layer are transitions rejected
 // by the exact same callback used by global A* and local DWA.
 for (int gy = 0; gy < grid.h(); ++gy) {
 for (int gx = 0; gx < grid.w(); ++gx) {
 const sim::Vec2 center = grid.gridToWorld(gx, gy);
 const auto current = terrain.sample(center);
 const auto draw_edge = [&](int nx, int ny, sim::Vec2 edge_a, sim::Vec2 edge_b) {
 if (!grid.inside(nx, ny))
 return;
 const sim::Vec2 neighbor = grid.gridToWorld(nx, ny);
 const auto adjacent = terrain.sample(neighbor);
 bool emphasize = false;
 Color color = Fade(kText, 0.42f);
 if (terrain_layer_ == TerrainLayer::Elevation) {
 emphasize = static_cast<int>(std::floor(current.height_m / 0.5 + 1e-6)) !=
 static_cast<int>(std::floor(adjacent.height_m / 0.5 + 1e-6));
 } else if (terrain_layer_ == TerrainLayer::FinalCost) {
 const double outward = mission.navigation_traversal_cost(center, neighbor);
 const double inward = mission.navigation_traversal_cost(neighbor, center);
 emphasize = !std::isfinite(outward) && !std::isfinite(inward);
 color = kDanger;
 }
 if (emphasize)
 DrawLineEx(world_to_screen(edge_a), world_to_screen(edge_b),
 terrain_layer_ == TerrainLayer::FinalCost ? 3.0f : 1.7f, Fade(color, 0.92f));
 };
 draw_edge(gx + 1, gy, {center.x + resolution * 0.5, center.y - resolution * 0.5},
 {center.x + resolution * 0.5, center.y + resolution * 0.5});
 draw_edge(gx, gy + 1, {center.x - resolution * 0.5, center.y + resolution * 0.5},
 {center.x + resolution * 0.5, center.y + resolution * 0.5});
 }
 }
}

void WorldRenderer::draw_terrain_layer_panel(const game::MissionSimulation& mission) const {
 const char* labels[] = {"world.layer_elevation", "world.layer_slope", "world.layer_risk",
 "world.layer_cost"};
 const float panel_width = std::min(510.0f, std::max(300.0f, viewport_.width - 370.0f));
 const Rectangle panel{viewport_.x + 18.0f, viewport_.y + 17.0f, panel_width, 68.0f};
 DrawRectangleRounded(panel, 0.12f, 10, {14, 18, 19, 238});
 DrawRectangleLinesEx(panel, 1.0f, Fade(kCyan, 0.25f));
 const float chip_width = (panel.width - 16.0f) / 4.0f;
 for (int index = 0; index < 4; ++index) {
 const bool active = index == static_cast<int>(terrain_layer_);
 const Rectangle chip{panel.x + 8.0f + chip_width * index, panel.y + 7.0f, chip_width - 4.0f, 25.0f};
 DrawRectangleRounded(chip, 0.42f, 8, active ? Color{28, 76, 72, 245} : Color{27, 31, 31, 218});
 if (active)
 DrawRectangleLinesEx(chip, 1.0f, Fade(kCyan, 0.72f));
 draw_text(catalog_.tr(labels[index]), {chip.x + 8.0f, chip.y + 5.0f}, panel.width < 400.0f ? 12 : 14,
 active ? kCyan : Fade(kMuted, 0.88f));
 }

 std::string detail;
 if (terrain_layer_ == TerrainLayer::Elevation) {
 double min_height = std::numeric_limits<double>::infinity();
 double max_height = -std::numeric_limits<double>::infinity();
 const auto& grid = mission.navigation_map();
 for (int gy = 0; gy < grid.h(); ++gy) {
 for (int gx = 0; gx < grid.w(); ++gx) {
 const double height = mission.terrain_model().sample(grid.gridToWorld(gx, gy)).height_m;
 min_height = std::min(min_height, height);
 max_height = std::max(max_height, height);
 }
 }
 detail = catalog_.trf("world.layer_elevation_detail", {fixed(min_height, 1), fixed(max_height, 1)});
 } else if (terrain_layer_ == TerrainLayer::Slope) {
 detail = catalog_.tr("world.layer_slope_detail");
 } else if (terrain_layer_ == TerrainLayer::VehicleRisk) {
 detail = catalog_.tr("world.layer_risk_detail");
 } else {
 detail = catalog_.tr("world.layer_cost_detail");
 }
 const float gradient_x = panel.x + 12.0f;
 const float gradient_y = panel.y + 45.0f;
 if (terrain_layer_ == TerrainLayer::Elevation) {
 DrawRectangleGradientH(static_cast<int>(gradient_x), static_cast<int>(gradient_y), 92, 7,
 Color{31, 55, 89, 255}, Color{247, 188, 92, 255});
 } else {
 DrawRectangleGradientH(static_cast<int>(gradient_x), static_cast<int>(gradient_y), 46, 7,
 Color{54, 185, 141, 255}, kAmber);
 DrawRectangleGradientH(static_cast<int>(gradient_x + 46.0f), static_cast<int>(gradient_y), 46, 7,
 kAmber, kDanger);
 }
 draw_text(detail, {panel.x + 113.0f, panel.y + 39.0f}, panel.width < 400.0f ? 11 : 14,
 Fade(kText, 0.84f));
}

void WorldRenderer::draw_dust(const game::MissionSimulation& mission) const {
 // Visual only: dust puffs behind the wheels on loose ground. Capped pool,
 // fading alpha, no physics in or out. Spawn rate follows speed × surface
 // dust factor (sand 1, gravel 0.5, else 0) — display density, not a model.
 const auto& state = mission.snapshot();
 const auto& map = mission.map_definition();
 double dust_factor = 0.0;
 for (const auto& surface : map.surfaces) {
  if (!game::active_in_level(surface.active_in, mission.level().id))
   continue;
  if (state.robot.x < surface.min.x || state.robot.x > surface.max.x || state.robot.y < surface.min.y ||
      state.robot.y > surface.max.y)
   continue;
  if (surface.material == "sand")
   dust_factor = std::max(dust_factor, 1.0);
  else if (surface.material == "gravel")
   dust_factor = std::max(dust_factor, 0.5);
 }
 const float dt = std::min(0.05f, GetFrameTime());
 for (auto& puff : dust_) {
  puff.age_s += dt;
 }
 dust_.erase(
   std::remove_if(dust_.begin(), dust_.end(), [](const DustPuff& puff) { return puff.age_s >= puff.life_s; }),
   dust_.end());
 const double speed = std::hypot(state.robot.v, state.robot.w * 0.5);
 if (dust_factor > 0.0 && std::abs(speed) > 0.8 && dust_.size() < 60 &&
     mission.snapshot().status == game::MissionStatus::Running) {
  static unsigned seed = 0xD05u;
  seed = seed * 1664525u + 1013904223u;
  const float jitter = static_cast<float>(seed % 100) / 100.0f - 0.5f;
  const float back = static_cast<float>(state.robot.theta) + 3.14159265f;
  DustPuff puff;
  puff.pos = {state.robot.x + std::cos(back) * 0.45 + jitter * 0.3,
              state.robot.y + std::sin(back) * 0.45 + jitter * 0.3};
  puff.life_s = 0.9f;
  puff.size_px = static_cast<float>((4.0 + dust_factor * 4.0) * scale_ / 64.0);
  dust_.push_back(puff);
 }
 for (const auto& puff : dust_) {
  const float t = puff.age_s / puff.life_s;
  const float fade = static_cast<float>((1.0 - t) * 0.55);
  const float size = puff.size_px * (0.6f + t * 1.6f);
  const Vector2 center = world_to_screen(puff.pos);
  if (dust_sprite_.id > 0) {
   const Rectangle source{0.0f, 0.0f, static_cast<float>(dust_sprite_.width),
                          static_cast<float>(dust_sprite_.height)};
   DrawTexturePro(dust_sprite_, source, {center.x, center.y, size * 2.0f, size * 2.0f}, {size, size},
                  t * 140.0f, {255, 255, 255, static_cast<unsigned char>(fade * 255)});
  } else {
   DrawCircleGradient(center, size, Fade({168, 158, 128, 255}, fade), {168, 158, 128, 0});
  }
 }
}

void WorldRenderer::draw_robot(const game::MissionSimulation& mission) const {
 const auto& state = mission.snapshot();
 const auto& robot = mission.robot();
 const auto center = world_to_screen({state.robot.x, state.robot.y});
 const Color mode = state.waiting_for_clearance ? kAmber : kCyan;
 const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(state.time_s * 5.5));
 const float body_radius = clampf(static_cast<float>(robot.collision_radius_m) * scale_, 28.0f, 43.0f);
 // Collision truth ring: exact physics radius, no visibility clamp. Body art
 // is drawn bigger than this (readability), so on close passes art may touch
 // a box while this ring stays clear — the ring arbitrates, not the art.
 {
  const float exact = static_cast<float>(robot.collision_radius_m) * scale_;
  DrawRing(center, exact - 1.2f, exact + 1.2f, 0, 360, 64, Fade(kText, 0.55f));
 }
 const double min_radius = mission.chassis_caps().geometry.minimum_turn_radius_m;
 if (min_radius > 0.0) {
  // Minimum turning circles, tangent to the heading on both sides. A
  // steering car can never place its center inside these discs — if the
  // route bends tighter than R, brake early and swing wide.
  const double cosine = std::cos(state.robot.theta);
  const double sine = std::sin(state.robot.theta);
  for (const double side : {-1.0, 1.0}) {
   const auto disc = world_to_screen({
     state.robot.x - sine * side * min_radius,
     state.robot.y + cosine * side * min_radius,
   });
   const float pixels = static_cast<float>(min_radius) * scale_;
   DrawRing(disc, pixels - 1.5f, pixels + 1.5f, 0, 360, 64, Fade(kAmber, 0.40f));
  }
  draw_text(catalog_.trf("world.min_radius", {fixed(min_radius, 2)}),
            {center.x + body_radius + 8, center.y - 8}, 15, Fade(kAmber, 0.85f));
 }

 const auto find_mount = [&](const std::string& id) -> const game::MountDefinition* {
  const auto found =
    std::find_if(robot.mounts.begin(), robot.mounts.end(), [&](const auto& mount) { return mount.id == id; });
  return found == robot.mounts.end() ? nullptr : &*found;
 };
 const auto component_id = [&](const std::string& slot) -> const std::string& {
  return mission.build().part(slot);
 };
 const auto mounted_position = [&](const game::MountDefinition& mount) {
  const double cosine = std::cos(state.robot.theta);
  const double sine = std::sin(state.robot.theta);
  return world_to_screen({
    state.robot.x + cosine * mount.position_m.x - sine * mount.position_m.y,
    state.robot.y + sine * mount.position_m.x + cosine * mount.position_m.y,
  });
 };
 const auto color = [](const game::VisualDefinition& visual) {
  return Color{visual.color[0], visual.color[1], visual.color[2], visual.color[3]};
 };

 struct RenderPart {
  const game::VisualDefinition* visual;
  const game::MountDefinition* mount;
 };
 std::vector<RenderPart> parts;
 for (const auto& part : robot.base_parts) {
  if (const auto* mount = find_mount(part.mount))
   parts.push_back({&part.visual, mount});
 }
 for (const auto& slot : robot.slots) {
  const auto& id = component_id(slot.slot);
  if (id.empty())
   continue;
  const auto& visual = catalog_.component(id).visual;
  for (const auto& mount_id : slot.mounts) {
 const bool mount_has_base_part = std::any_of(robot.base_parts.begin(), robot.base_parts.end(),
 [&](const auto& part) { return part.mount == mount_id; });
 if (!mount_has_base_part) {
 if (const auto* mount = find_mount(mount_id))
    parts.push_back({&visual, mount});
 }
  }
 }
 std::sort(parts.begin(), parts.end(), [](const auto& left, const auto& right) {
  return left.mount->position_m.z < right.mount->position_m.z;
 });

 DrawEllipse(static_cast<int>(center.x + 8), static_cast<int>(center.y + 11), body_radius * 1.08f,
             body_radius * 0.82f, {3, 4, 4, 155});
 draw_soft_disc(center, body_radius + 18 + pulse * 3, mode);

 for (const auto& part : parts) {
  const auto& visual = *part.visual;
  const auto part_center = mounted_position(*part.mount);
  // Screen frame is y-flipped: world-CCW (theta, delta) renders as screen
  // rotation -(theta + delta). Body uses -theta; steered front wheels
  // subtract their linkage angle the same way. Positive delta (left turn)
  // reads counterclockwise on screen, matching the world turn.
  float angle = static_cast<float>(-state.robot.theta * 180.0 / sim::kPi - part.mount->rotation_deg.z);
 const auto actuation = std::find_if(state.control.mount_actuations.begin(),
 state.control.mount_actuations.end(),
 [&](const auto& candidate) { return candidate.mount_id == part.mount->id; });
 if (actuation != state.control.mount_actuations.end()) {
 angle -= static_cast<float>(actuation->yaw_rad * 180.0 / sim::kPi);
  }
  const float width = std::max(5.0f, static_cast<float>(visual.size_m.x) * scale_);
  const float height = std::max(5.0f, static_cast<float>(visual.size_m.y) * scale_);
  const Color fill = color(visual);
  if (visual.primitive_2d == "disc") {
   const float radius = std::max(width, height) * 0.5f;
   DrawCircleV({part_center.x + 3, part_center.y + 4}, radius + 3, {4, 5, 5, 175});
   DrawCircleV(part_center, radius, fill);
   DrawRing(part_center, radius * 0.48f, radius * 0.68f, 0, 360, 24, {22, 34, 33, 230});
   DrawCircleV(part_center, std::max(2.0f, radius * 0.22f), {220, 244, 231, 255});
  } else {
   DrawRectanglePro({part_center.x + 4, part_center.y + 6, width + 2, height + 2},
                    {width * 0.5f, height * 0.5f}, angle, {0, 3, 8, 170});
   DrawRectanglePro({part_center.x, part_center.y, width, height}, {width * 0.5f, height * 0.5f}, angle,
                    fill);
   if (visual.primitive_2d == "wheel") {
    DrawRectanglePro({part_center.x, part_center.y, width * 0.62f, height * 0.48f},
                     {width * 0.31f, height * 0.24f}, angle, {68, 72, 68, 220});
   } else {
    DrawRectanglePro({part_center.x, part_center.y, width * 0.50f, height * 0.52f},
                     {width * 0.25f, height * 0.26f}, angle, {42, 53, 51, 210});
   }
  }
 }

 const float heading = static_cast<float>(state.robot.theta);
 const Vector2 nose = polar(center, body_radius * 0.95f, heading);
 const Vector2 eye_left = polar(nose, body_radius * 0.34f, heading + 1.5708f);
 const Vector2 eye_right = polar(nose, body_radius * 0.34f, heading - 1.5708f);
 const Vector2 beam_left = polar(eye_left, body_radius * 0.65f, heading);
 const Vector2 beam_right = polar(eye_right, body_radius * 0.65f, heading);
 DrawLineEx(eye_left, beam_left, 5, Fade(mode, 0.12f));
 DrawLineEx(eye_right, beam_right, 5, Fade(mode, 0.12f));
 DrawLineEx(eye_left, eye_right, 10.0f, {29, 39, 37, 255});
 DrawCircleV(eye_left, 3.8f, {230, 245, 211, 255});
 DrawCircleV(eye_right, 3.8f, {230, 245, 211, 255});
 DrawLineEx(polar(center, body_radius * 0.42f, heading + 2.25f),
            polar(center, body_radius * 0.42f, heading - 2.25f), 2.0f, Fade(kText, 0.25f));

 const std::string status = state.waiting_for_clearance
                              ? catalog_.tr("world.status_waiting")
                              : catalog_.trf("world.status_go", {mission.level().objective.goal_label});
 const auto measured = MeasureTextEx(font_, status.c_str(), 13, 0.25f);
 const Rectangle label{center.x - measured.x * 0.5f - 11, center.y + body_radius + 17, measured.x + 22, 28};
 DrawRectangleRounded(label, 0.42f, 9, {21, 25, 23, 235});
 DrawTriangle({center.x - 5, label.y}, {center.x + 5, label.y}, {center.x, label.y - 6}, {21, 25, 23, 235});
 draw_text(status, {label.x + 11, label.y + 6}, 16, kText);
}

void WorldRenderer::draw_telemetry(const game::MissionSimulation& mission, int screen_height) const {
 const float y = std::min(viewport_.y + viewport_.height - 47.0f, static_cast<float>(screen_height) - 52.0f);
 const std::string summary = catalog_.trf("world.summary", {fixed(mission.snapshot().time_s, 0),
                                                            fixed(mission.result().path_length_m, 1),
                                                            fixed(mission.result().energy_wh, 2)});
 const auto measured = MeasureTextEx(font_, summary.c_str(), 13, 0.25f);
 const Rectangle pill{viewport_.x + 18, y, measured.x + 30, 30};
 DrawRectangleRounded(pill, 0.46f, 10, {17, 20, 19, 220});
 draw_text(summary, {pill.x + 15, pill.y + 7}, 16, Fade(kText, 0.78f));
 if (analysis_mode_) {
  const auto& state = mission.snapshot();
  const std::string terrain =
    catalog_.trf("world.terrain_summary",
                 {fixed(std::atan(state.terrain_grade) * kRadToDeg, 1),
                  fixed(std::atan(state.cross_slope) * kRadToDeg, 1), fixed(state.rollover_risk * 100.0, 0)});
  const auto terrain_measured = MeasureTextEx(font_, terrain.c_str(), 13, 0.25f);
  const Rectangle terrain_pill{viewport_.x + 18, y - 36.0f, terrain_measured.x + 30, 30};
  DrawRectangleRounded(terrain_pill, 0.46f, 10, {17, 20, 19, 220});
  draw_text(terrain, {terrain_pill.x + 15, terrain_pill.y + 7}, 16,
            mission.snapshot().rollover_risk >= 0.75 ? kDanger : Fade(kText, 0.78f));
 const std::string strategy = catalog_.trf("world.strategy_summary",
 {state.strategy_policy, fixed(state.strategy.applied.speed_scale * 100.0, 0),
 state.strategy.overridden ? catalog_.tr("world.strategy_guarded")
 : catalog_.tr("world.strategy_clean")});
 const auto strategy_measured = MeasureTextEx(font_, strategy.c_str(), 13, 0.25f);
 const Rectangle strategy_pill{viewport_.x + 18, y - 72.0f, strategy_measured.x + 30, 30};
 DrawRectangleRounded(strategy_pill, 0.46f, 10, {17, 20, 19, 220});
 draw_text(strategy, {strategy_pill.x + 15, strategy_pill.y + 7}, 16,
 state.strategy.overridden ? kAmber : Fade(kText, 0.78f));
 }
}

} // namespace client
