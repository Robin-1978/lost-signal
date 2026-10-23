// SIM-side live viewer (2D top-down). Reads truth + recorded episode state
// through ScenarioSession (SIM composition root) and draws it with raylib.
// Never publishes to ROBOT: the session steps headless exactly as without
// a window; the viewer only observes. Headless CI never runs this binary
// (needs a display); frame logic is unit-tested in test_viewer_frame.

#include <raylib.h>
#include <rlgl.h>

#include <cmath>
#include <cstdio>
#include <deque>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/map_editor.hpp"
#include "sim/viewer/view_transform.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/terrain.hpp"

namespace {
using lost_signal::sim::viewer::Frame;
using lost_signal::sim::viewer::ViewTransform;

constexpr int kScreenW = 1100;
constexpr int kScreenH = 750;

// Ground-material paint (master's drive_effect colors, SIM-side display
// only — physics never reads these; unknown ids fall back to asphalt).
Color material_color(const std::string& id, unsigned char alpha = 255) {
  if (id == "standing_water") return {0x7B, 0xCE, 0xD2, alpha};
  if (id == "sand") return {0xD8, 0xB9, 0x78, alpha};
  if (id == "gravel") return {0xB3, 0xA1, 0x7A, alpha};
  if (id == "mud") return {0x79, 0x60, 0x44, alpha};
  if (id == "paved") return {0x89, 0x91, 0x8B, alpha};
  if (id == "grip_grating") return {0x4E, 0xE7, 0xC5, alpha};
  if (id == "steel_deck") return {0xB8, 0xC4, 0xCA, alpha};
  if (id == "ice") return {0xC4, 0xE8, 0xF2, alpha};
  if (id == "asphalt") return {0x52, 0x64, 0x6A, alpha};
  return {0x52, 0x64, 0x6A, alpha};  // weathered_asphalt fallback
}

// Terrain height tint (low grass -> dry earth -> rock).
Color height_color(double h, double hmax) {
  const double t = hmax > 1e-6 ? std::min(1.0, std::max(0.0, h / hmax)) : 0.0;
  if (t < 0.5) {
    const float k = static_cast<float>(t * 2.0);
    return {static_cast<unsigned char>(126 + 110 * k),
            static_cast<unsigned char>(178 - 20 * k),
            static_cast<unsigned char>(120 - 30 * k), 255};
  }
  const float k = static_cast<float>((t - 0.5) * 2.0);
  return {static_cast<unsigned char>(236 - 60 * k),
          static_cast<unsigned char>(158 - 50 * k),
          static_cast<unsigned char>(90 - 10 * k), 255};
}

// Flat fake shadow: a dark translucent disc under bodies/props. No shadow
// map, no shader — 80% of the depth cue for 1% of the cost.
inline void blob_shadow_3d(double x_m, double y_m, double r_m) {
  DrawCylinder({static_cast<float>(x_m), 0.012f,
                static_cast<float>(-y_m)},
               static_cast<float>(r_m), static_cast<float>(r_m), 0.012f, 20,
               {20, 20, 25, 50});
}

// Actors sidecar from tools/convert_master_map.py (<map>.actors.json):
// patrol/dwell -> waypoints (+waits), seeded-wander -> wander. Missing
// file = no actors (demo legacy), never an error.
void load_actors_sidecar(const std::string& map_path,
                         lost_signal::sim::scenario::ScenarioSession& session) {
  const std::string::size_type dot = map_path.rfind('.');
  const std::string side =
      (dot == std::string::npos ? map_path : map_path.substr(0, dot)) +
      ".actors.json";
  std::ifstream in(side, std::ios::binary);
  if (!in.is_open()) return;
  std::ostringstream ss;
  ss << in.rdbuf();
  std::string err;
  auto root = lost_signal::robot::core::json_parse(ss.str(), &err);
  if (root == nullptr || !root->is_object()) return;
  const auto* arr = root->find("actors");
  if (arr == nullptr || !arr->is_array()) return;
  int loaded = 0;
  for (const auto& e : arr->as_array()) {
    if (!e.is_object()) continue;
    const auto* wx = e.find("x");
    const auto* wy = e.find("y");
    if (wx == nullptr || wy == nullptr || !wx->is_num() || !wy->is_num()) {
      continue;
    }
    lost_signal::sim::world::Actor a;
    a.x = wx->as_num();
    a.y = wy->as_num();
    const auto* r = e.find("radius_m");
    if (r != nullptr && r->is_num()) a.radius_m = r->as_num();
    const auto* h = e.find("height_m");
    if (h != nullptr && h->is_num()) a.height_m = h->as_num();
    const auto* beh = e.find("behavior");
    const std::string kind =
        (beh != nullptr && beh->is_str()) ? beh->as_str() : "waypoints";
    if (!lost_signal::sim::world::ActorBehaviorRegistry::global().create(
            kind == "wander" ? "wander" : "waypoints", a.behavior)) {
      continue;
    }
    const auto* sp = e.find("speed_mps");
    if (sp != nullptr && sp->is_num()) a.behavior.speed_mps = sp->as_num();
    const auto* lp = e.find("loop");
    if (lp != nullptr && lp->is_bool()) a.behavior.loop = lp->as_bool();
    const auto* sd = e.find("seed");
    if (sd != nullptr && sd->is_num()) {
      a.behavior.seed = static_cast<std::uint64_t>(sd->as_num());
    }
    const auto* wps = e.find("waypoints");
    if (wps != nullptr && wps->is_array()) {
      for (const auto& w : wps->as_array()) {
        const auto* qx = w.find("x");
        const auto* qy = w.find("y");
        if (qx == nullptr || qy == nullptr || !qx->is_num() ||
            !qy->is_num()) {
          continue;
        }
        a.behavior.waypoints.push_back({qx->as_num(), qy->as_num()});
      }
    }
    const auto* wt = e.find("waits");
    if (wt != nullptr && wt->is_array()) {
      for (const auto& v : wt->as_array()) {
        if (v.is_num()) a.behavior.waits_s.push_back(v.as_num());
      }
    }
    session.setup_world().add_actor(a);
    ++loaded;
  }
  std::fprintf(stderr, "actors sidecar: %s (%d actors)\n", side.c_str(),
               loaded);
}

// Map extents over every authored layer (auto-fit on --map load).
void map_bounds(const lost_signal::sim::world::World& w, double& x0,
                double& y0, double& x1, double& y1) {
  x0 = y0 = 1e9;
  x1 = y1 = -1e9;
  auto eat = [&](double x, double y) {
    x0 = std::min(x0, x);
    y0 = std::min(y0, y);
    x1 = std::max(x1, x);
    y1 = std::max(y1, y);
  };
  eat(0.0, 0.0);
  for (const auto& b : w.static_boxes()) {
    eat(b.x0, b.y0);
    eat(b.x1, b.y1);
  }
  for (const auto& p : w.places()) eat(p.x_m, p.y_m);
  for (const auto& z : w.material_zones()) {
    eat(z.x0, z.y0);
    eat(z.x1, z.y1);
  }
  for (const auto& p : w.terrain_patches()) {
    eat(p.x0, p.y0);
    eat(p.x1, p.y1);
  }
  if (x1 - x0 < 8.0) {
    x0 -= (8.0 - (x1 - x0)) * 0.5;
    x1 += (8.0 - (x1 - x0)) * 0.5;
  }
  if (y1 - y0 < 8.0) {
    y0 -= (8.0 - (y1 - y0)) * 0.5;
    y1 += (8.0 - (y1 - y0)) * 0.5;
  }
}

void setup_demo(lost_signal::sim::scenario::ScenarioSession& session) {
  session.setup_world().set_static_boxes({
      {5.2, 1.0, 6.0, 1.8, 0.0},
      {2.5, 5.0, 3.3, 5.8, 0.0},
      {-2.4, 2.0, -1.6, 2.8, 0.0},
      {1.0, -2.4, 1.8, -1.6, 0.0},
  });
  session.setup_world().add_place({"loopA", 4.0, 0.0, 0.0});
  session.setup_world().add_place({"loopB", 4.0, 4.0, 0.0});
  session.setup_world().add_place({"loopC", 0.0, 4.0, 0.0});
  session.setup_world().add_place({"loopD", 0.0, 0.0, 0.0});
  session.set_objectives({"loopA", "loopB", "loopC", "loopD"});
  lost_signal::robot::iface::TaskRequest req;
  req.request_id = "view";
  req.place_id = "loopA";
  session.request_goal(req);
}

Color box_color(const lost_signal::sim::viewer::FrameBox& b,
                bool collision_flash) {
  if (b.selected) return YELLOW;
  if (collision_flash) return RED;
  return DARKGRAY;
}

void draw_frame_2d(const Frame& f, const ViewTransform& view, bool show_lidar,
                   bool show_estimate) {
  // Ground grid, 1 m spacing over the visible range.
  double wx0 = 0.0, wy0 = 0.0, wx1 = 0.0, wy1 = 0.0;
  if (view.screen_to_world(0, 0, wx0, wy0) &&
      view.screen_to_world(static_cast<float>(GetScreenWidth()),
                           static_cast<float>(GetScreenHeight()), wx1, wy1)) {
    const double gx0 = std::floor(std::min(wx0, wx1));
    const double gx1 = std::ceil(std::max(wx0, wx1));
    const double gy0 = std::floor(std::min(wy0, wy1));
    const double gy1 = std::ceil(std::max(wy0, wy1));
    for (double gx = gx0; gx <= gx1; gx += 1.0) {
      const auto a = view.world_to_screen(gx, gy0);
      const auto b = view.world_to_screen(gx, gy1);
      DrawLineV({a.x_px, a.y_px}, {b.x_px, b.y_px}, {220, 220, 220, 255});
    }
    for (double gy = gy0; gy <= gy1; gy += 1.0) {
      const auto a = view.world_to_screen(gx0, gy);
      const auto b = view.world_to_screen(gx1, gy);
      DrawLineV({a.x_px, a.y_px}, {b.x_px, b.y_px}, {220, 220, 220, 255});
    }
  }
  // Ground-material paint + terrain tint (SIM truth overlay, same colors
  // as the 3D frontend below).
  for (const auto& z : f.zones) {
    const auto a = view.world_to_screen(z.x0_m, z.y0_m);
    const auto c = view.world_to_screen(z.x1_m, z.y1_m);
    const float x = std::min(a.x_px, c.x_px), y = std::min(a.y_px, c.y_px);
    const float w = std::abs(c.x_px - a.x_px), h = std::abs(c.y_px - a.y_px);
    DrawRectangle(x, y, w, h, material_color(z.material_id, 70));
  }
  for (const auto& p : f.patches) {
    const auto a = view.world_to_screen(p.x0_m, p.y0_m);
    const auto c = view.world_to_screen(p.x1_m, p.y1_m);
    const float x = std::min(a.x_px, c.x_px), y = std::min(a.y_px, c.y_px);
    const float w = std::abs(c.x_px - a.x_px), h = std::abs(c.y_px - a.y_px);
    DrawRectangle(x, y, w, h, {190, 150, 100, 60});
    DrawRectangleLines(x, y, w, h, {150, 110, 70, 160});
  }
  // Trail (truth history, oldest first).
  for (std::size_t i = 1; i < f.trail.size(); ++i) {
    const auto a = view.world_to_screen(f.trail[i - 1].first,
                                        f.trail[i - 1].second);
    const auto b =
        view.world_to_screen(f.trail[i].first, f.trail[i].second);
    DrawLineV({a.x_px, a.y_px}, {b.x_px, b.y_px}, BLUE);
  }
  // Boxes.
  for (const auto& b : f.boxes) {
    const auto a = view.world_to_screen(b.x0_m, b.y0_m);
    const auto c = view.world_to_screen(b.x1_m, b.y1_m);
    const float x = std::min(a.x_px, c.x_px), y = std::min(a.y_px, c.y_px);
    const float w = std::abs(c.x_px - a.x_px), h = std::abs(c.y_px - a.y_px);
    DrawRectangle(x, y, w, h, box_color(b, f.static_collision));
    DrawRectangleLines(x, y, w, h, BLACK);
  }
  // Places (green when the objective driver reports done).
  for (const auto& p : f.places) {
    const auto s = view.world_to_screen(p.x_m, p.y_m);
    const Color col = p.reached ? GREEN : ORANGE;
    DrawCircleV({s.x_px, s.y_px}, 7.0f, col);
    DrawCircleLines(s.x_px, s.y_px, 7.0f, BLACK);
    const float hx = s.x_px + 12.0f * std::cos(static_cast<float>(p.yaw_rad));
    const float hy = s.y_px - 12.0f * std::sin(static_cast<float>(p.yaw_rad));
    DrawLineV({s.x_px, s.y_px}, {hx, hy}, BLACK);
    DrawText(p.place_id.c_str(), static_cast<int>(s.x_px) + 10,
             static_cast<int>(s.y_px) - 18, 12, DARKGRAY);
  }
  // Actors.
  for (const auto& a : f.actors) {
    const auto s = view.world_to_screen(a.x_m, a.y_m);
    DrawCircleV({s.x_px, s.y_px},
                static_cast<float>(a.radius_m * view.scale()), PURPLE);
    DrawCircleLines(s.x_px, s.y_px,
                    static_cast<float>(a.radius_m * view.scale()), BLACK);
  }
  // Editor selection handles: box corners + selected place ring.
  for (const auto& b : f.boxes) {
    if (!b.selected) continue;
    const double cx[4] = {b.x0_m, b.x1_m, b.x1_m, b.x0_m};
    const double cy[4] = {b.y0_m, b.y0_m, b.y1_m, b.y1_m};
    for (int c = 0; c < 4; ++c) {
      const auto s = view.world_to_screen(cx[c], cy[c]);
      DrawCircleV({s.x_px, s.y_px}, c == b.selected_corner ? 7.0f : 5.0f,
                  c == b.selected_corner ? RED : YELLOW);
      DrawCircleLines(s.x_px, s.y_px, c == b.selected_corner ? 7.0f : 5.0f,
                      BLACK);
    }
  }
  for (const auto& p : f.places) {
    if (!p.selected) continue;
    const auto s = view.world_to_screen(p.x_m, p.y_m);
    DrawCircleLines(s.x_px, s.y_px, 12.0f, RED);
  }
  // Lidar hits (from the true sensor origin, same as the 3D fan).
  if (show_lidar && f.lidar_valid) {
    const auto o = view.world_to_screen(f.body.sens_x_m, f.body.sens_y_m);
    for (const auto& hit : f.lidar_hits) {
      const auto s = view.world_to_screen(hit.first, hit.second);
      DrawLineV({o.x_px, o.y_px}, {s.x_px, s.y_px}, {255, 0, 0, 60});
      DrawPixelV({s.x_px, s.y_px}, RED);
    }
  }
  // ROBOT estimate ghost (offline truth-vs-estimate overlay, live).
  if (show_estimate && f.body.estimate_valid) {
    const auto s = view.world_to_screen(f.body.est_x_m, f.body.est_y_m);
    DrawCircleLines(s.x_px, s.y_px, 9.0f, DARKGREEN);
    const float hx =
        s.x_px + 14.0f * std::cos(static_cast<float>(f.body.est_yaw_rad));
    const float hy =
        s.y_px - 14.0f * std::sin(static_cast<float>(f.body.est_yaw_rad));
    DrawLineV({s.x_px, s.y_px}, {hx, hy}, DARKGREEN);
  }
  // Body (truth rect + heading).
  {
    const auto s = view.world_to_screen(f.body.x_m, f.body.y_m);
    const float hl =
        static_cast<float>(f.body.half_length_m * view.scale());
    const float hw = static_cast<float>(f.body.half_width_m * view.scale());
    const float yaw = static_cast<float>(f.body.yaw_rad);
    const float c = std::cos(yaw), sn = std::sin(yaw);
    Vector2 corners[4];
    const float lx[4] = {hl, hl, -hl, -hl};
    const float ly[4] = {hw, -hw, -hw, hw};
    for (int i = 0; i < 4; ++i) {
      const float rx = lx[i] * c - ly[i] * sn;
      const float ry = lx[i] * sn + ly[i] * c;
      corners[i] = {s.x_px + rx, s.y_px - ry};
    }
    DrawTriangleFan(corners, 4, f.static_collision ? RED : SKYBLUE);
    for (int i = 0; i < 4; ++i) {
      DrawLineV(corners[i], corners[(i + 1) % 4], DARKBLUE);
    }
    const float hx = s.x_px + (hl + 10.0f) * c;
    const float hy = s.y_px - (hl + 10.0f) * sn;
    DrawLineV({s.x_px, s.y_px}, {hx, hy}, DARKBLUE);
  }
  // Wheels (simplified, default on): short segments at the module
  // positions, steered by the live joint (ackermann fronts read visibly),
  // red while the engine flags traction saturation (per side on tracks).
  // Screen is y-flipped: world angle t draws along (cos t, -sin t).
  {
    const float yaw = static_cast<float>(f.body.yaw_rad);
    const float c = std::cos(yaw), sn = std::sin(yaw);
    const float sc = static_cast<float>(view.scale());
    const auto bs = view.world_to_screen(f.body.x_m, f.body.y_m);
    auto to_screen = [&](double bx, double by) {
      return Vector2{bs.x_px + static_cast<float>(bx) * c * sc -
                         static_cast<float>(by) * sn * sc,
                     bs.y_px - (static_cast<float>(bx) * sn * sc +
                               static_cast<float>(by) * c * sc)};
    };
    if (f.body.tracked) {
      // Skid-steer slabs: side lines, red per side from the drive flags.
      for (int k = 0; k < 2; ++k) {
        const double wy = (k == 0 ? 1.0 : -1.0) * f.body.half_width_m;
        const Vector2 a = to_screen(-f.body.half_length_m, wy);
        const Vector2 b = to_screen(f.body.half_length_m, wy);
        const bool slip = k < static_cast<int>(f.body.wheels.size()) &&
                          f.body.wheels[static_cast<std::size_t>(k)].slipping;
        DrawLineEx(a, b, 3.0f, slip ? RED : DARKGRAY);
      }
    } else {
      for (const auto& w : f.body.wheels) {
        const Vector2 m = to_screen(w.x_m, w.y_m);
        const float t =
            yaw + static_cast<float>(w.steer_rad);  // steered heading
        const float half =
            static_cast<float>(w.radius_m * 2.0 * sc * 0.5);
        const Vector2 a{m.x - half * std::cos(t),
                        m.y + half * std::sin(t)};
        const Vector2 b{m.x + half * std::cos(t),
                        m.y - half * std::sin(t)};
        DrawLineEx(a, b, 3.0f, w.slipping ? RED : DARKGRAY);
      }
    }
  }
}

// 3D frontend over the same Frame model. World (x, y, z_up) maps to raylib
// (x, z_up, -y): +y north goes away from the default camera, right-handed.
struct CameraState {
  double target_x = 2.0, target_y = 1.0;
  float yaw_deg = 35.0f;    // orbit around the target
  float pitch_deg = 50.0f;  // elevation above the ground plane
  float dist_m = 12.0f;
};

inline Vector3 to_raylib(double x_m, double y_m, double z_m) {
  return {static_cast<float>(x_m), static_cast<float>(z_m),
          static_cast<float>(-y_m)};
}

inline Camera3D camera_from_state(const CameraState& st) {
  const float yaw = st.yaw_deg * DEG2RAD;
  const float pitch = st.pitch_deg * DEG2RAD;
  Vector3 pos{static_cast<float>(st.target_x + st.dist_m * std::cos(pitch) * std::cos(yaw)),
              static_cast<float>(st.dist_m * std::sin(pitch)),
              static_cast<float>(-(st.target_y + st.dist_m * std::cos(pitch) * std::sin(yaw)))};
  return {.position = pos,
          .target = to_raylib(st.target_x, st.target_y, 0.0),
          .up = {0.0f, 1.0f, 0.0f},
          .fovy = 45.0f,
          .projection = CAMERA_PERSPECTIVE};
}

// Ground-plane pick for 3D edit mode: intersect the mouse ray with z_up=0.
inline bool screen_to_ground_3d(const Camera3D& cam, Vector2 mouse,
                                double& x_m, double& y_m) {
  const Ray ray = GetMouseRay(mouse, cam);
  if (std::abs(ray.direction.y) < 1e-6f) return false;
  const float t = -ray.position.y / ray.direction.y;
  if (t < 0.0f) return false;
  x_m = ray.position.x + t * ray.direction.x;
  y_m = -(ray.position.z + t * ray.direction.z);
  return true;
}

void draw_frame_3d(const Frame& f,
                   const lost_signal::sim::world::World& world,
                   bool show_lidar, bool show_estimate) {
  namespace world_ns = lost_signal::sim::world;
  // Ground base sized to the authored map (concrete apron under everything).
  double gx0, gy0, gx1, gy1;
  map_bounds(world, gx0, gy0, gx1, gy1);
  {
    const float w = static_cast<float>(gx1 - gx0 + 8.0);
    const float h = static_cast<float>(gy1 - gy0 + 8.0);
    DrawPlane(to_raylib((gx0 + gx1) * 0.5, (gy0 + gy1) * 0.5, -0.02),
              {w, h}, {198, 203, 196, 255});
  }
  DrawGrid(static_cast<int>(std::max(gx1 - gx0, gy1 - gy0) + 8.0), 1.0f);
  // Material zones as translucent paint quads (slick water reads blue,
  // sand/gravel warm — same table as the 2D overlay).
  for (const auto& z : f.zones) {
    const Vector3 center = to_raylib((z.x0_m + z.x1_m) * 0.5,
                                     (z.y0_m + z.y1_m) * 0.5, 0.005);
    const Vector3 size{static_cast<float>(std::abs(z.x1_m - z.x0_m)), 0.006f,
                       static_cast<float>(std::abs(z.y1_m - z.y0_m))};
    DrawCubeV(center, size, material_color(z.material_id, 150));
  }
  // Terrain patches as a height-tinted mesh sampling the SAME canonical
  // function the engine builds and the survey rasterizes — the slope you
  // see is the slope the robot climbs.
  {
    double hmax = 0.2;
    for (const auto& p : f.patches) {
      for (double x = p.x0_m; x < p.x1_m; x += 0.5) {
        for (double y = p.y0_m; y < p.y1_m; y += 0.5) {
          const auto s = world_ns::sample_terrain(
              world.terrain_patches(), world.support_slabs(), x, y);
          hmax = std::max(hmax, s.height_m);
        }
      }
    }
    for (const auto& p : f.patches) {
      for (double x = p.x0_m; x < p.x1_m; x += 0.5) {
        for (double y = p.y0_m; y < p.y1_m; y += 0.5) {
          const double x2 = std::min(x + 0.5, p.x1_m);
          const double y2 = std::min(y + 0.5, p.y1_m);
          const double h00 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x, y)
                                 .height_m;
          const double h10 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x2, y)
                                 .height_m;
          const double h01 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x, y2)
                                 .height_m;
          const double h11 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x2, y2)
                                 .height_m;
          const double hm = (h00 + h10 + h01 + h11) * 0.25 + 0.012;
          const Color col = height_color(hm, hmax);
          // Winding faces up in raylib space (x, z_up, -y): culling eats
          // downward triangles, so p00 -> p10 -> p01, never the reverse.
          DrawTriangle3D(to_raylib(x, y, h00 + 0.012),
                         to_raylib(x2, y, h10 + 0.012),
                         to_raylib(x, y2, h01 + 0.012), col);
          DrawTriangle3D(to_raylib(x2, y, h10 + 0.012),
                         to_raylib(x2, y2, h11 + 0.012),
                         to_raylib(x, y2, h01 + 0.012), col);
        }
      }
    }
  }
  // Trail at ground level.
  for (std::size_t i = 1; i < f.trail.size(); ++i) {
    DrawLine3D(to_raylib(f.trail[i - 1].first, f.trail[i - 1].second, 0.02),
               to_raylib(f.trail[i].first, f.trail[i].second, 0.02), BLUE);
  }
  // Boxes as two-tone blocks (light cap, dark walls) with blob shadows.
  for (const auto& b : f.boxes) {
    const float h = static_cast<float>(b.height_m > 0.02 ? b.height_m : 0.02);
    const Vector3 center =
        to_raylib((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5, h * 0.5);
    const Vector3 size{static_cast<float>(std::abs(b.x1_m - b.x0_m)), h,
                       static_cast<float>(std::abs(b.y1_m - b.y0_m))};
    const Color wall =
        b.selected ? YELLOW
                   : (f.static_collision ? RED
                                         : Color{110, 118, 128, 255});
    DrawCubeV(center, size, wall);
    DrawCubeWiresV(center, size, BLACK);
    // Sun-lit cap: thin slab on top, warm white.
    DrawCubeV(to_raylib((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5,
                        h + 0.008),
              {size.x, 0.016f, size.z},
              b.selected ? Color{255, 240, 160, 255}
                         : Color{168, 175, 184, 255});
    blob_shadow_3d((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5,
                   std::max(size.x, size.z) * 0.75);
    if (b.selected) {
      const double cx[4] = {b.x0_m, b.x1_m, b.x1_m, b.x0_m};
      const double cy[4] = {b.y0_m, b.y0_m, b.y1_m, b.y1_m};
      for (int c = 0; c < 4; ++c) {
        DrawSphere(to_raylib(cx[c], cy[c], h),
                   c == b.selected_corner ? 0.12f : 0.08f,
                   c == b.selected_corner ? RED : YELLOW);
      }
    }
  }
  // Places as beacon poles with a ground ring (green = objective done).
  for (const auto& p : f.places) {
    const Color col = p.reached ? GREEN : ORANGE;
    DrawCylinder(to_raylib(p.x_m, p.y_m, 0.3), 0.05f, 0.05f, 0.6f, 10,
                 DARKGRAY);
    DrawSphere(to_raylib(p.x_m, p.y_m, 0.68), 0.13f, col);
    DrawCircle3D(to_raylib(p.x_m, p.y_m, 0.02), 0.45f, {1, 0, 0}, 90.0f,
                 p.reached ? Color{40, 180, 80, 200} : Color{230, 140, 40, 200});
    if (p.selected) DrawSphereWires(to_raylib(p.x_m, p.y_m, 0.68), 0.25f, 8,
                                    8, RED);
  }
  // Actors: forklift-orange vehicles, pedestrian-blue walkers, true height,
  // heading tick, blob shadow. Patrol routes draw as ground polylines.
  for (const auto& a : f.actors) {
    const Color body =
        a.pedestrian ? Color{70, 130, 220, 255} : Color{235, 130, 40, 255};
    const float h = static_cast<float>(a.height_m > 0.1 ? a.height_m : 0.5);
    blob_shadow_3d(a.x_m, a.y_m, a.radius_m * 1.3);
    DrawCylinder(to_raylib(a.x_m, a.y_m, h * 0.5), static_cast<float>(h),
                 static_cast<float>(h),
                 static_cast<float>(a.radius_m), 16, body);
    DrawCylinder(to_raylib(a.x_m, a.y_m, h + 0.03), 0.06f, 0.06f, 0.12f, 8,
                 DARKGRAY);  // roof beacon
    DrawLine3D(to_raylib(a.x_m, a.y_m, h * 0.5),
               to_raylib(a.x_m + a.vx_mps, a.y_m + a.vy_mps, h * 0.5), BLACK);
  }
  for (const auto& a : world.actors()) {
    if (a.behavior.kind !=
            lost_signal::sim::world::ActorBehaviorKind::kWaypoints ||
        a.behavior.waypoints.empty()) {
      continue;
    }
    Vector3 prev = to_raylib(a.x, a.y, 0.03);
    for (const auto& wp : a.behavior.waypoints) {
      const Vector3 cur = to_raylib(wp.x_m, wp.y_m, 0.03);
      DrawLine3D(prev, cur, {150, 100, 220, 180});
      prev = cur;
    }
    if (a.behavior.loop && a.behavior.waypoints.size() > 1) {
      DrawLine3D(prev, to_raylib(a.behavior.waypoints.front().x_m,
                                 a.behavior.waypoints.front().y_m, 0.03),
                 {150, 100, 220, 180});
    }
  }
  // Lidar rays from the TRUE sensor origin (mount-composed, same source
  // generation runs). Endpoints stay planar-projected (see builder).
  if (show_lidar && f.lidar_valid) {
    const Vector3 o = to_raylib(f.body.sens_x_m, f.body.sens_y_m,
                                f.body.sens_z_m);
    for (const auto& hit : f.lidar_hits) {
      DrawLine3D(o, to_raylib(hit.first, hit.second, f.body.sens_z_m),
                 {255, 0, 0, 90});
    }
  }
  // Estimate ghost: wireframe box at the ROBOT pose.
  if (show_estimate && f.body.estimate_valid) {
    DrawCubeWiresV(
        to_raylib(f.body.est_x_m, f.body.est_y_m, 0.25),
        {static_cast<float>(f.body.half_length_m * 2.0), 0.5f,
         static_cast<float>(f.body.half_width_m * 2.0)},
        DARKGREEN);
  }
  // Body: box at engine pose with live attitude. Exact world->raylib
  // rotation M = Q*R*Q' for R = Rz(yaw)*Ry(pitch)*Rx(roll) — no Euler
  // folklore, slopes tilt the viz exactly as the engine holds them. (This
  // also fixes a latent mirror: the old -yaw drew the nose south for a
  // north truth; actors/spheres carry no heading so nobody noticed.)
  {
    const double cr = std::cos(f.body.roll_rad),
                 sr = std::sin(f.body.roll_rad);
    const double cp = std::cos(f.body.pitch_rad),
                 sp = std::sin(f.body.pitch_rad);
    const double cy = std::cos(f.body.yaw_rad),
                 sy = std::sin(f.body.yaw_rad);
    // R = Rz*Ry*Rx, row-major.
    const double r00 = cy * cp, r01 = cy * sp * sr - sy * cr,
                 r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp, r11 = sy * sp * sr + cy * cr,
                 r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp, r21 = cp * sr, r22 = cp * cr;
    // M = Q*R*Q' with Q:(x,y,z)->(x,z,-y); column-major for rlMultMatrixf.
    float m[16] = {
        static_cast<float>(r00), static_cast<float>(r20),
        static_cast<float>(-r10), 0.0f,
        static_cast<float>(r02), static_cast<float>(r22),
        static_cast<float>(-r12), 0.0f,
        static_cast<float>(-r01), static_cast<float>(-r21),
        static_cast<float>(r11), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f};
    blob_shadow_3d(f.body.x_m, f.body.y_m,
                   std::max(f.body.half_length_m, f.body.half_width_m) * 1.4);
    rlPushMatrix();
    rlTranslatef(static_cast<float>(f.body.x_m),
                 static_cast<float>(f.body.z_m + 0.25),
                 static_cast<float>(-f.body.y_m));
    rlMultMatrixf(m);
    {
      // Layered shop model (local frame, y up): charcoal skirt, pearl
      // hull, orange deck stripe, lidar mast + puck, headlights, status
      // LED (red while any wheel slips). Collision still flashes red.
      const float bl = static_cast<float>(f.body.half_length_m * 2.0);
      const float bw = static_cast<float>(f.body.half_width_m * 2.0);
      const Color hull = f.static_collision
                             ? RED
                             : Color{238, 241, 244, 255};
      bool slipping = false;
      for (const auto& sw : f.body.wheels) slipping = slipping || sw.slipping;
      DrawCube({0, -0.20f, 0}, bl, 0.12f, bw, Color{45, 48, 55, 255});
      DrawCube({0, -0.03f, 0}, bl, 0.24f, bw, hull);
      DrawCubeWires({0, -0.03f, 0}, bl, 0.24f, bw, DARKBLUE);
      DrawCube({0, 0.105f, 0}, bl * 0.7f, 0.03f, bw * 0.55f,
               Color{235, 120, 35, 255});  // deck stripe
      const float mx = -bl * 0.32f;
      DrawCylinder({mx, 0.26f, 0}, 0.035f, 0.035f, 0.3f, 10,
                   Color{45, 48, 55, 255});  // mast
      DrawCylinder({mx, 0.44f, 0}, 0.075f, 0.075f, 0.07f, 14,
                   Color{200, 208, 215, 255});  // lidar puck
      DrawSphere({mx, 0.50f, 0}, 0.025f,
                 slipping ? RED : Color{50, 200, 90, 255});  // status LED
      DrawSphere({bl * 0.5f, -0.02f, bw * 0.30f}, 0.035f,
                 Color{255, 242, 200, 255});  // headlights
      DrawSphere({bl * 0.5f, -0.02f, -bw * 0.30f}, 0.035f,
                 Color{255, 242, 200, 255});
    }
    // Wheels from live joints (drive spin + steer yaw per module, same
    // sources as physics/encoders). A spoke makes spin visible; slipping
    // wheels draw red from the engine traction flags. Mecanum modules get
    // static 45-degree tread stripes (rollers are analytic-wrench state,
    // never meshed — honest decoration, documented in the frame).
    // Skid-steer bases draw track slabs instead (wheels hide under tracks,
    // like the real machine); slab color per side from the same flags.
    if (f.body.tracked) {
      // Track slabs sized from the chassis params; per-side color from
      // the traction flags (wheels[0] = left drive). Local Z = -bodyY, so
      // the left slab draws at negative Z (mirroring would swap the red
      // sides — pinned by the drive convention, not by symmetry).
      const float slab_len =
          static_cast<float>(f.body.half_length_m * 2.0f * 0.9f);
      const float slab_z =
          static_cast<float>(0.06 - f.body.z_m - 0.25);
      for (int s = -1; s <= 1; s += 2) {
        const bool slip = s > 0 ? (f.body.wheels.size() > 1 &&
                                   f.body.wheels[1].slipping)
                                : (f.body.wheels.size() > 0 &&
                                   f.body.wheels[0].slipping);
        DrawCube({0.0f, slab_z,
                  static_cast<float>(s * (f.body.half_width_m - 0.075))},
                 slab_len, 0.12f, 0.15f, slip ? RED : DARKGRAY);
      }
    } else {
      // Draw-local axes are raylib-world (Y up, Z = -bodyY): steer yaws
      // about Y (truth + = left, verified by the ackermann left-arc pin),
      // the axle lies lateral (Z) so the cylinder rolls 90 deg about X
      // first, spin follows about the laid-down axle. Spoke + slip-red
      // from live joints/flags; mecanum tread stripes sit in the steer
      // frame (rollers are passive analytic state — static decoration).
      for (const auto& w : f.body.wheels) {
        const float axle =
            static_cast<float>(w.radius_m - f.body.z_m - 0.25);
        rlPushMatrix();
        rlTranslatef(static_cast<float>(w.x_m), axle,
                     static_cast<float>(-w.y_m));
        rlRotatef(static_cast<float>(w.steer_rad * RAD2DEG), 0.0f, 1.0f,
                  0.0f);
        rlRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        if (w.mecanum_rollers) {
          for (int k = -1; k <= 1; ++k) {
            rlPushMatrix();
            rlTranslatef(static_cast<float>(k) * 0.03f, 0.0f, 0.0f);
            rlRotatef(45.0f, 0.0f, 0.0f, 1.0f);
            DrawCube({0.0f, 0.0f, 0.0f}, 0.012f, 0.02f,
                     2.0f * static_cast<float>(w.radius_m), DARKGRAY);
            rlPopMatrix();
          }
        }
        rlRotatef(static_cast<float>(w.spin_rad * RAD2DEG), 0.0f, 1.0f,
                  0.0f);
        const Color tire =
            w.slipping ? RED : Color{40, 40, 40, 255};
        DrawCylinder({0.0f, 0.0f, 0.0f}, static_cast<float>(w.radius_m),
                     static_cast<float>(w.radius_m), 0.08f, 12, tire);
        DrawCube({0.0f, 0.0f, 0.0f}, 2.0f * static_cast<float>(w.radius_m),
                 0.02f, 0.03f, YELLOW);  // spoke: spin reads visually
        rlPopMatrix();
      }
    }
    rlPopMatrix();
  }
}
}  // namespace

int main(int argc, char** argv) {
  using namespace lost_signal;
  // Optional single-source configuration: --robot <config> builds the
  // session from one RobotConfig file (ROBOT description + SIM motor
  // numbers), --map <world file> replaces the demo geometry.
  std::string robot_path, sim_path, map_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--robot" && i + 1 < argc) robot_path = argv[++i];
    if (a == "--sim" && i + 1 < argc) sim_path = argv[++i];
    if (a == "--map" && i + 1 < argc) map_path = argv[++i];
  }
  std::string used_config, config_err;
  auto robot_config = robot::platform::load_default_robot_config(
      robot_path, &used_config, &config_err);
  if (!robot_path.empty() && !robot_config.has_value()) {
    std::fprintf(stderr, "bad --robot file: %s\n", config_err.c_str());
    return 1;
  }
  std::unique_ptr<sim::scenario::ScenarioSession> session;
  if (robot_config.has_value()) {
    std::fprintf(stderr, "robot config: %s\n", used_config.c_str());
    session = std::make_unique<sim::scenario::ScenarioSession>(
        robot::platform::description_from_config(*robot_config),
        sim::eval::motor_from_config(*robot_config));
  } else {
    session = std::make_unique<sim::scenario::ScenarioSession>();
  }
  // Applied lidar mount for the viz ray fan: read back from the session
  // (the exact extrinsics generation runs — never a separately authored
  // copy, so a misconfigured mount shows up instead of hiding).
  sim::viewer::LidarMount session_lidar_mount;
  session->lidar_mount(session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
  std::string used_sim, sim_err;
  auto sim_config = sim::eval::load_default_sim_config(sim_path, &used_sim,
                                                       &sim_err);
  if (!sim_path.empty() && !sim_config.has_value()) {
    std::fprintf(stderr, "bad --sim file: %s\n", sim_err.c_str());
    return 1;
  }
  if (sim_config.has_value()) {
    std::fprintf(stderr, "sim config: %s\n", used_sim.c_str());
    session->apply_sim_config(*sim_config);
  }
  if (!map_path.empty()) {
    sim::eval::WorldFileContent content;
    std::string err;
    if (!sim::eval::load_world_file(map_path, content, &err)) {
      std::fprintf(stderr, "bad --map file: %s\n", err.c_str());
      return 1;
    }
    session->setup_world().set_static_boxes(std::move(content.boxes));
    std::vector<std::string> ids;
    for (const auto& p : content.places) ids.push_back(p.place_id);
    session->setup_world().set_places(std::move(content.places));
    session->setup_world().set_materials(std::move(content.materials));
    session->setup_world().set_material_zones(std::move(content.zones));
    session->setup_world().set_terrain_patches(std::move(content.patches));
    session->setup_world().set_support_slabs(std::move(content.slabs));
    session->setup_world().set_environment(std::move(content.environment));
    session->set_objectives(std::move(ids));
    // Survey covers the authored map (far-field goals need a route).
    session->fit_map_window_to_world();
    // Converted master maps ship patrols beside the world file.
    load_actors_sidecar(map_path, *session);
  } else {
    setup_demo(*session);
  }

  InitWindow(kScreenW, kScreenH, "robot_simu viewer (SIM truth, 2D)");
  SetTargetFPS(60);

  sim::viewer::ViewTransform view(50.0f, 2.0, 1.0, kScreenW, kScreenH);
  view.fit(-4.0, -4.0, 8.0, 7.0);
  std::deque<std::pair<double, double>> trail;
  bool paused = false;
  bool show_lidar = true;
  bool show_estimate = true;
  bool follow = true;
  int steps_per_frame = 4;
  Vector2 last_mouse{0, 0};
  bool dragging = false;
  // Map editor: entering edit mode snapshots the world and pauses the sim;
  // leaving keeps the applied geometry.
  sim::viewer::MapEditor editor;
  bool edit_mode = false;
  std::string edit_msg;
  // 3D orbit camera over the same frame (V toggles 2D/3D, --3d starts in 3D).
  bool view3d = false;
  // --shot N path: render N frames, screenshot, exit (visual smoke check).
  int shot_frames = -1;
  std::string shot_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--3d") view3d = true;
    if (a == "--shot" && i + 2 < argc) {
      shot_frames = std::atoi(argv[++i]);
      shot_path = argv[++i];
    }
  }
  CameraState cam_st;
  if (!map_path.empty()) {
    // Frame the authored map instead of the demo window.
    double bx0, by0, bx1, by1;
    map_bounds(session->setup_world(), bx0, by0, bx1, by1);
    view.fit(bx0 - 2.0, by0 - 2.0, bx1 + 2.0, by1 + 2.0);
    cam_st.target_x = (bx0 + bx1) * 0.5;
    cam_st.target_y = (by0 + by1) * 0.5;
    cam_st.dist_m = static_cast<float>(
        std::min(60.0, std::max(10.0, std::max(bx1 - bx0, by1 - by0) * 0.9)));
    follow = false;  // start on the map overview, not glued to the robot
  }

  while (!WindowShouldClose()) {
    view.set_screen(GetScreenWidth(), GetScreenHeight());
    if (IsKeyPressed(KEY_V)) view3d = !view3d;
    // Edit-mode picking: 2D uses the view transform, 3D the ground ray.
    const Vector2 mpos = GetMousePosition();
    double pick_x = 0.0, pick_y = 0.0;
    bool on_map = false;
    if (view3d) {
      const Camera3D pick_cam =
          camera_from_state(cam_st);
      on_map = screen_to_ground_3d(pick_cam, mpos, pick_x,
                                                pick_y);
    } else {
      on_map = view.screen_to_world(mpos.x, mpos.y, pick_x, pick_y);
    }
    // Edit-mode toggle + keys (window thread only).
    if (IsKeyPressed(KEY_M)) {
      edit_mode = !edit_mode;
      if (edit_mode) {
        editor.load_from(session->setup_world());
        paused = true;
      } else {
        editor.apply(session->setup_world());
      }
    }
    if (edit_mode) {
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && on_map) {
        editor.press(pick_x, pick_y);
      }
      if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && on_map) {
        editor.drag(pick_x, pick_y);
      }
      if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) editor.release();
      if (IsKeyPressed(KEY_B) && on_map) editor.add_box(pick_x, pick_y);
      if (IsKeyPressed(KEY_P) && on_map) editor.add_place(pick_x, pick_y);
      if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
        editor.remove_selected();
      }
      if (IsKeyPressed(KEY_S)) {
        editor.save("viewer_map.json");
        edit_msg = editor.message();
      }
      // Live preview: paused sim, so pushing every dirty frame costs only
      // an engine model rebuild per edit, no stepping hazard.
      if (editor.dirty()) editor.apply(session->setup_world());
    }
    // Input (window thread only; sim stays deterministic).
    if (IsKeyPressed(KEY_SPACE)) paused = !paused;
    if (IsKeyPressed(KEY_N) && paused) session->step_once();
    if (IsKeyPressed(KEY_L)) show_lidar = !show_lidar;
    if (IsKeyPressed(KEY_E)) show_estimate = !show_estimate;
    if (IsKeyPressed(KEY_F)) follow = !follow;
    if (IsKeyPressed(KEY_EQUAL) && steps_per_frame < 40) steps_per_frame *= 2;
    if (IsKeyPressed(KEY_MINUS) && steps_per_frame > 1) steps_per_frame /= 2;
    if (IsKeyPressed(KEY_ZERO)) view.fit(-4.0, -4.0, 8.0, 7.0);
    const float wheel = GetMouseWheelMove();
    if (view3d) {
      if (wheel != 0.0f) {
        cam_st.dist_m *= wheel > 0.0f ? 1.0f / 1.15f : 1.15f;
        if (cam_st.dist_m < 2.0f) cam_st.dist_m = 2.0f;
        if (cam_st.dist_m > 60.0f) cam_st.dist_m = 60.0f;
      }
    } else if (wheel != 0.0f) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      view.zoom_at(GetMouseX(), GetMouseY(),
                   wheel > 0.0f ? 1.15f : 1.0f / 1.15f);
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Vector2 m = GetMousePosition();
      if (dragging) {
        if (view3d) {
          cam_st.yaw_deg += (m.x - last_mouse.x) * 0.3f;
          cam_st.pitch_deg += (last_mouse.y - m.y) * 0.3f;
          if (cam_st.pitch_deg < 10.0f) cam_st.pitch_deg = 10.0f;
          if (cam_st.pitch_deg > 85.0f) cam_st.pitch_deg = 85.0f;
        } else {
          view.set_screen(GetScreenWidth(), GetScreenHeight());
          view.pan_px(last_mouse.x - m.x, last_mouse.y - m.y);
        }
        follow = false;
      }
      last_mouse = m;
      dragging = true;
    } else {
      dragging = false;
    }

    if (!paused && !session->objectives_done()) {
      for (int i = 0; i < steps_per_frame; ++i) {
        session->step_once();
        if (session->objectives_done()) break;
      }
    }
    const auto& steps = session->episode().steps();
    if (steps.empty()) continue;
    const sim::eval::EvalStep& last = steps.back();
    trail.emplace_back(last.truth_after.body_x_m, last.truth_after.body_y_m);
    while (trail.size() > 600) trail.pop_front();
    std::vector<std::string> reached;
    for (const auto& r : session->objectives().results()) {
      if (r.robot_done) reached.push_back(r.place_id);
    }
    const sim::viewer::Frame frame = sim::viewer::build_frame(
        session->setup_world(), last, reached,
        {trail.begin(), trail.end()},
        edit_mode ? sim::viewer::EditorSelection{editor.selected_box(),
                                                 editor.selected_corner(),
                                                 editor.selected_place()}
                  : sim::viewer::EditorSelection{},
        session_lidar_mount);
    if (follow) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      // Re-center without touching zoom: pan by the screen-space error.
      const auto cur =
          view.world_to_screen(last.truth_after.body_x_m,
                               last.truth_after.body_y_m);
      view.pan_px(GetScreenWidth() * 0.5f - cur.x_px,
                  GetScreenHeight() * 0.5f - cur.y_px);
      cam_st.target_x = last.truth_after.body_x_m;
      cam_st.target_y = last.truth_after.body_y_m;
    }

    BeginDrawing();
    ClearBackground(view3d ? Color{168, 204, 228, 255} : RAYWHITE);
    if (view3d) {
      const Camera3D cam = camera_from_state(cam_st);
      BeginMode3D(cam);
      draw_frame_3d(frame, session->setup_world(), show_lidar, show_estimate);
      EndMode3D();
    } else {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      draw_frame_2d(frame, view, show_lidar, show_estimate);
    }
    char hud[256];
    std::snprintf(hud, sizeof(hud),
                  "step %llu  x=%.2f y=%.2f yaw=%.2f  clr=%.2f  obj %d/4  %dx%d %s",
                  static_cast<unsigned long long>(frame.step), frame.body.x_m,
                  frame.body.y_m, frame.body.yaw_rad, frame.static_clearance_m,
                  static_cast<int>(reached.size()), steps_per_frame,
                  GetFPS(), paused ? "PAUSED" : "RUN");
    DrawText(hud, 10, 10, 14, BLACK);
    if (edit_mode) {
      char ebuf[256];
      std::snprintf(ebuf, sizeof(ebuf),
                    "EDIT  click/drag move  B box  P place  Del remove  S save viewer_map.json  M exit  %s",
                    edit_msg.c_str());
      DrawText(ebuf, 10, 32, 14, RED);
    }
    DrawText("space pause  N step  L lidar  E estimate  F follow  0 fit  +/- speed  V 2D/3D  M edit  right-drag pan/orbit  wheel zoom",
             10, GetScreenHeight() - 24, 12, DARKGRAY);
    EndDrawing();
    if (shot_frames > 0 && --shot_frames == 0) {
      TakeScreenshot(shot_path.c_str());
      break;
    }
  }
  CloseWindow();
  return 0;
}
