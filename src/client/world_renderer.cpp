#include "world_renderer.hpp"

#include "scene_renderer_3d.hpp"
#include "game/content.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
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

float clampf(float value, float low, float high) {
 return std::max(low, std::min(value, high));
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
 scene_3d_ = std::make_unique<SceneRenderer3D>(assets_root, catalog);
}

WorldRenderer::~WorldRenderer() {
 if (owns_font_)
  UnloadFont(font_);
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
                         const std::string& chapter_title) {
 ClearBackground(kVoid);
 DrawRectangleGradientV(0, 0, screen_width, screen_height, {24, 25, 24, 255}, kVoid);

 const float available_width = std::max(320.0f, static_cast<float>(screen_width) - 360.0f);
 viewport_ = {18.0f, 76.0f, available_width - 36.0f,
              std::max(360.0f, static_cast<float>(screen_height) - 104.0f)};

 if (!analysis_mode_) {
  scene_3d_->draw(mission, screen_width, screen_height);
  draw_header(mission, static_cast<int>(available_width), chapter_title);
  draw_telemetry(mission, screen_height);
  draw_view_mode();
  return;
 }

 const auto map_size = mission.map_definition().size_m;
 const bool active = mission.snapshot().status == game::MissionStatus::Running;
 if (active) {
  scale_ = clampf(std::min(viewport_.width / 12.2f, viewport_.height / 8.3f), 50.0f, 92.0f);
  sim::Vec2 focus{mission.snapshot().robot.x, mission.snapshot().robot.y};
  if (!mission.snapshot().dynamic_obstacles.empty()) {
   const auto& detected = mission.snapshot().dynamic_obstacles.front();
   focus.x = mission.snapshot().robot.x * 0.58 + detected.estimated_position.x * 0.42;
   focus.y = mission.snapshot().robot.y * 0.58 + detected.estimated_position.y * 0.42 + 0.80;
  } else {
   focus.x += 1.2 * std::cos(mission.snapshot().robot.theta);
   focus.y += 1.2 * std::sin(mission.snapshot().robot.theta);
  }
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

void WorldRenderer::draw_view_mode() const {
 const std::string text = analysis_mode_ ? "AI 认知视图  ·  Tab 返回现场" : "探索视图  ·  Tab 查看 AI 认知";
 const auto measured = MeasureTextEx(font_, text.c_str(), 13, 0.25f);
 const Rectangle pill{21.0f, 70.0f, measured.x + 26.0f, 29.0f};
 DrawRectangleRounded(pill, 0.46f, 10, analysis_mode_ ? Color{24, 61, 64, 235} : Color{16, 20, 19, 205});
 draw_text(text, {pill.x + 13.0f, pill.y + 6.0f}, 13, analysis_mode_ ? kCyan : Fade(kText, 0.82f));
}

void WorldRenderer::draw_header(const game::MissionSimulation& mission, int width,
                                const std::string& chapter_title) const {
 draw_text(chapter_title + "  /  第一章", {22, 14}, 13, Fade(kMuted, 0.88f));
 draw_text(mission.level().title, {21, 35}, 26, kText);

 const auto status = mission.snapshot().status;
 const char* text = "等待出发";
 Color color = kAmber;
 if (status == game::MissionStatus::Running) {
  text = mission.level().control_mode == "manual_calibration" ? "底盘校准由你控制" : "洛一正在自主行动";
  color = kCyan;
 } else if (status == game::MissionStatus::Succeeded) {
  text = "信号已经恢复";
  color = kCyan;
 } else if (status == game::MissionStatus::Failed) {
  text = "行动中断";
  color = kDanger;
 }
 const auto measured = MeasureTextEx(font_, text, 14, 0.25f);
 const Rectangle badge{static_cast<float>(width) - measured.x - 43, 23, measured.x + 25, 31};
 DrawRectangleRounded(badge, 0.45f, 12, {18, 21, 21, 218});
 DrawCircleV({badge.x + 13, badge.y + 15.5f}, 4.0f, color);
 draw_text(text, {badge.x + 23, badge.y + 7}, 14, kText);
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
 if (const auto goal = map.markers.find(mission.level().objective.goal); goal != map.markers.end()) {
  draw_soft_disc(world_to_screen({goal->second.x, goal->second.y}), scale_ * 1.55f, {63, 159, 148, 255});
 }

 const auto map_top_left = world_to_screen({0, map.size_m.y});
 const auto map_bottom_right = world_to_screen({map.size_m.x, 0});
 DrawRectangleLinesEx(
   {map_top_left.x, map_top_left.y, map_bottom_right.x - map_top_left.x, map_bottom_right.y - map_top_left.y},
   5, {67, 65, 57, 210});
 for (const auto& surface : map.surfaces) {
  const auto top_left = world_to_screen({surface.min.x, surface.max.y});
  const auto bottom_right = world_to_screen({surface.max.x, surface.min.y});
  Rectangle box{top_left.x, top_left.y, bottom_right.x - top_left.x, bottom_right.y - top_left.y};
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
  draw_text(surface.label, {box.x + 34, box.y + 15}, 12, Fade(kText, 0.78f));
 }
 for (const auto& actor : map.actors) {
  if (!game::active_in_level(actor.active_in, mission.level().id) || actor.path.size() < 2)
   continue;
  const auto start = world_to_screen(actor.path.front());
  const auto end = world_to_screen(actor.path.back());
  const auto middle = lerp(start, end, 0.5f);
  DrawLineEx({start.x, start.y - 38}, {end.x, end.y - 38}, 2.0f, {198, 161, 91, 42});
  DrawLineEx({start.x, start.y + 38}, {end.x, end.y + 38}, 2.0f, {198, 161, 91, 42});
  draw_text("叉车通道", {middle.x - 35, middle.y + 52}, 14, {198, 181, 137, 62});
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
 const auto draw_marker = [&](const std::string& id, Color color, const std::string& label, bool tower) {
  const auto found = map.markers.find(id);
  if (found == map.markers.end())
   return;
  const auto position = world_to_screen({found->second.x, found->second.y});
  draw_soft_disc(position, 46 + pulse * 7, color);
  DrawCircleV({position.x + 5, position.y + 8}, 21, {7, 8, 8, 135});
  DrawCircleV(position, 18, tower ? Color{62, 66, 62, 255} : Color{83, 72, 54, 255});
  DrawRing(position, 15, 18, 0, 360, 48, {111, 108, 91, 255});
  if (tower) {
   DrawCircleV(position, 8, {23, 31, 30, 255});
   DrawCircleV(position, 4.5f + pulse * 1.5f, color);
   DrawRing(position, 25 + pulse * 5, 26 + pulse * 5, 0, 360, 64, Fade(color, 0.28f));
  } else {
   DrawRectangleRounded({position.x - 9, position.y - 8, 18, 16}, 0.18f, 4, {170, 115, 54, 255});
   DrawRectangle(static_cast<int>(position.x - 2), static_cast<int>(position.y - 6), 4, 12,
                 {237, 188, 83, 255});
  }
  const auto measured = MeasureTextEx(font_, label.c_str(), 13, 0.25f);
  const Rectangle tag{position.x - measured.x * 0.5f - 9, position.y + 27, measured.x + 18, 24};
  DrawRectangleRounded(tag, 0.42f, 8, {17, 20, 19, 215});
  draw_text(label, {tag.x + 9, tag.y + 5}, 13, kText);
 };
 if (!mission.level().objective.via.empty()) {
  draw_marker(mission.level().objective.via, kAmber, mission.level().objective.via_label, false);
 }
 draw_marker(mission.level().objective.goal, kCyan, mission.level().objective.goal_label, true);

 const auto robot_position = world_to_screen({
   mission.snapshot().robot.x,
   mission.snapshot().robot.y,
 });
 std::size_t ray_index = 0;
 for (const auto& hit : mission.snapshot().lidar_hits) {
  const std::size_t stride = mission.snapshot().dynamic_object_detected ? 4 : 8;
  if (ray_index++ % stride != 0)
   continue;
  const auto point = world_to_screen(hit);
  DrawLineEx(robot_position, point, 1, {82, 222, 199, 24});
  DrawCircleV(point, 1.5f, {124, 236, 211, 110});
 }

 for (const auto& obstacle : mission.snapshot().dynamic_obstacles) {
  const auto estimated = world_to_screen(obstacle.estimated_position);
  const auto position = estimated;
  const float radius = clampf(static_cast<float>(obstacle.radius_m) * scale_, 25.0f, 42.0f);
  const Color alert = kDanger;
  const auto future = world_to_screen({
    obstacle.estimated_position.x + obstacle.estimated_velocity.x * 2.5,
    obstacle.estimated_position.y + obstacle.estimated_velocity.y * 2.5,
  });
  DrawLineEx(estimated, future, 8, Fade(alert, 0.08f));
  DrawLineEx(estimated, future, 2, Fade(alert, 0.45f));
  DrawCircleV(future, radius * 0.78f, Fade(alert, 0.08f));
  DrawRing(future, radius * 0.76f, radius * 0.81f, 0, 360, 40, Fade(alert, 0.32f));
  DrawRing(estimated, radius * 0.96f, radius, 0, 360, 40, Fade(kCyan, 0.72f));
  draw_soft_disc(position, radius + 22 + pulse * 4, alert);
  const float actor_angle = static_cast<float>(
    std::atan2(-obstacle.estimated_velocity.y, obstacle.estimated_velocity.x) * 180.0 / sim::kPi);
  const float actor_radians = -actor_angle * static_cast<float>(sim::kPi) / 180.0f;
  const auto front = polar(position, radius * 0.94f, actor_radians);
  const auto fork_left = polar(front, radius * 0.37f, actor_radians + 1.5708f);
  const auto fork_right = polar(front, radius * 0.37f, actor_radians - 1.5708f);
  const auto fork_left_end = polar(fork_left, radius * 0.82f, actor_radians);
  const auto fork_right_end = polar(fork_right, radius * 0.82f, actor_radians);
  DrawLineEx({position.x + 7, position.y + 10}, {front.x + 7, front.y + 10}, radius * 1.35f, {4, 5, 5, 135});
  DrawRectanglePro({position.x + 6, position.y + 8, radius * 1.75f, radius * 1.22f},
                   {radius * 0.88f, radius * 0.61f}, actor_angle, {5, 6, 6, 165});
  DrawRectanglePro({position.x, position.y, radius * 1.70f, radius * 1.16f}, {radius * 0.85f, radius * 0.58f},
                   actor_angle, {217, 139, 42, 255});
  DrawRectanglePro({position.x, position.y, radius * 0.76f, radius * 0.86f}, {radius * 0.38f, radius * 0.43f},
                   actor_angle, {50, 59, 58, 255});
  DrawRectanglePro({position.x, position.y, radius * 0.49f, radius * 0.60f},
                   {radius * 0.245f, radius * 0.30f}, actor_angle, {107, 139, 135, 220});
  DrawLineEx(fork_left, fork_left_end, std::max(3.0f, radius * 0.16f), {83, 82, 72, 255});
  DrawLineEx(fork_right, fork_right_end, std::max(3.0f, radius * 0.16f), {83, 82, 72, 255});
  DrawCircleV(polar(position, radius * 0.50f, actor_radians + 1.5708f), radius * 0.13f, {24, 25, 23, 255});
  DrawCircleV(polar(position, radius * 0.50f, actor_radians - 1.5708f), radius * 0.13f, {24, 25, 23, 255});
  DrawCircleV(position, 3.0f + pulse * 1.4f, {255, 211, 106, 255});
  const float separation =
    distance(estimated, robot_position) / scale_ - static_cast<float>(obstacle.radius_m) - 0.38f;
  const auto middle = lerp(robot_position, estimated, 0.52f);
  DrawLineEx(robot_position, estimated, 2.0f, Fade(kDanger, 0.27f));
  DrawCircleV(middle, 5, kDanger);
  const std::string warning = "路线冲突 · " + fixed(separation, 1) + " 米";
  const auto measured = MeasureTextEx(font_, warning.c_str(), 13, 0.25f);
  const Rectangle tag{estimated.x + radius + 8, estimated.y - radius - 10, measured.x + 18, 27};
  DrawRectangleRounded(tag, 0.38f, 8, {53, 24, 24, 232});
  draw_text(warning, {tag.x + 9, tag.y + 6}, 13, {255, 191, 178, 255});
 }

 draw_robot(mission);
 EndScissorMode();

 DrawRectangleRounded({viewport_.x + 18, viewport_.y + 17, 321, 29}, 0.48f, 10, {18, 21, 20, 215});
 DrawCircleV({viewport_.x + 33, viewport_.y + 31.5f}, 4, kCyan);
 draw_text("A* 全局路线  ·  DWA 局部轨迹  ·  动态局部图", {viewport_.x + 44, viewport_.y + 23}, 13,
           Fade(kText, 0.88f));

 const float card_width = 328.0f;
 const Rectangle card{
   viewport_.x + viewport_.width - card_width - 18.0f,
   viewport_.y + 17.0f,
   card_width,
   174.0f,
 };
 DrawRectangleRounded(card, 0.08f, 10, {14, 18, 19, 235});
 DrawRectangleLinesEx(card, 1.0f, Fade(control.dwa_active ? kAmber : kCyan, 0.34f));
 draw_text("控制诊断", {card.x + 15, card.y + 12}, 15, kText);
 const std::string planner = control.dwa_active
                               ? "DWA 活跃 · " + std::to_string(control.dwa_candidates.size()) + " 条候选"
                               : "DWA 待机 · 跟随 A* 路线";
 draw_text(planner, {card.x + 15, card.y + 36}, 12, control.dwa_active ? kAmber : Fade(kCyan, 0.82f));
 draw_text("上层航向 e " + fixed(control.heading_error_rad, 2) + " rad · v " +
             fixed(control.target_v_mps, 2) + " · ω " + fixed(control.target_w_radps, 2),
           {card.x + 15, card.y + 58}, 12, Fade(kText, 0.82f));
 draw_text("左右轮 · 目标/实测", {card.x + 15, card.y + 80}, 11, Fade(kMuted, 0.82f));
 draw_text("L 目标 " + fixed(control.left_target_rpm, 0) + " / 实测 " + fixed(control.left_measured_rpm, 0) +
             " rpm · PWM " + fixed(control.left_pwm * 100.0, 0) + "%",
           {card.x + 15, card.y + 99}, 12, Fade(kText, 0.86f));
 draw_text("R 目标 " + fixed(control.right_target_rpm, 0) + " / 实测 " +
             fixed(control.right_measured_rpm, 0) + " rpm · PWM " + fixed(control.right_pwm * 100.0, 0) + "%",
           {card.x + 15, card.y + 130}, 12, Fade(kText, 0.86f));
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
 draw_pwm(card.y + 148, control.right_pwm);
}

void WorldRenderer::draw_robot(const game::MissionSimulation& mission) const {
 const auto& state = mission.snapshot();
 const auto& robot = catalog_.robot(catalog_.campaign().companion_robot);
 const auto center = world_to_screen({state.robot.x, state.robot.y});
 const Color mode = state.waiting_for_clearance ? kAmber : kCyan;
 const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(state.time_s * 5.5));
 const float body_radius = clampf(static_cast<float>(robot.collision_radius_m) * scale_, 28.0f, 43.0f);

 const auto find_mount = [&](const std::string& id) -> const game::MountDefinition* {
  const auto found =
    std::find_if(robot.mounts.begin(), robot.mounts.end(), [&](const auto& mount) { return mount.id == id; });
  return found == robot.mounts.end() ? nullptr : &*found;
 };
 const auto component_id = [&](const std::string& slot) -> const std::string& {
  if (slot == "tires")
   return mission.build().tires;
  if (slot == "sensor")
   return mission.build().sensor;
  if (slot == "motor")
   return mission.build().motor;
  if (slot == "imu")
   return mission.build().imu;
  return mission.build().computer;
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
   if (const auto* mount = find_mount(mount_id))
    parts.push_back({&visual, mount});
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
  const float angle = static_cast<float>(-state.robot.theta * 180.0 / sim::kPi - part.mount->rotation_deg.z);
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

 const std::string status =
   state.waiting_for_clearance ? "我先等它通过" : "前往" + mission.level().objective.goal_label;
 const auto measured = MeasureTextEx(font_, status.c_str(), 13, 0.25f);
 const Rectangle label{center.x - measured.x * 0.5f - 11, center.y + body_radius + 17, measured.x + 22, 28};
 DrawRectangleRounded(label, 0.42f, 9, {21, 25, 23, 235});
 DrawTriangle({center.x - 5, label.y}, {center.x + 5, label.y}, {center.x, label.y - 6}, {21, 25, 23, 235});
 draw_text(status, {label.x + 11, label.y + 6}, 13, kText);
}

void WorldRenderer::draw_telemetry(const game::MissionSimulation& mission, int screen_height) const {
 const float y = std::min(viewport_.y + viewport_.height - 47.0f, static_cast<float>(screen_height) - 52.0f);
 const std::string summary = "已行动 " + fixed(mission.snapshot().time_s, 0) + " 秒" + "   ·   距离 " +
                             fixed(mission.result().path_length_m, 1) + " 米" + "   ·   电量消耗 " +
                             fixed(mission.result().energy_wh, 2) + " Wh";
 const auto measured = MeasureTextEx(font_, summary.c_str(), 13, 0.25f);
 const Rectangle pill{viewport_.x + 18, y, measured.x + 30, 30};
 DrawRectangleRounded(pill, 0.46f, 10, {17, 20, 19, 220});
 draw_text(summary, {pill.x + 15, pill.y + 7}, 13, Fade(kText, 0.78f));
}

} // namespace client
