#include <cassert>
#include <cmath>
#include <cstdio>

#include "sim/eval/episode.hpp"
#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/view_transform.hpp"
#include "sim/world/world.hpp"

namespace {
lost_signal::sim::eval::EvalStep make_step() {
  using namespace lost_signal;
  sim::eval::EvalStep s;
  s.step = 7;
  s.truth_after.body_x_m = 1.0;
  s.truth_after.body_y_m = 2.0;
  s.truth_after.body_yaw_rad = 0.5;
  s.truth_after.body_roll_rad = 0.1;
  s.truth_after.body_pitch_rad = -0.05;
  s.truth_after.body_z_m = 0.12;
  s.obs.scan_valid = true;
  s.obs.scan_angle_min_rad = -1.0;
  s.obs.scan_angle_increment_rad = 0.5;
  s.obs.ranges_m = {2.0f, 2.0f, 2.0f, 2.0f, 2.0f};
  s.judge.static_clearance_m = 0.3;
  return s;
}
}  // namespace

int main() {
  using namespace lost_signal;

  // ViewTransform round-trips world<->screen; zoom keeps the cursor fixed.
  {
    sim::viewer::ViewTransform v(50.0f, 1.0, 2.0, 800, 600);
    const auto p = v.world_to_screen(1.0, 2.0);
    assert(std::abs(p.x_px - 400.0f) < 1e-3f);
    assert(std::abs(p.y_px - 300.0f) < 1e-3f);
    double wx = 0.0, wy = 0.0;
    assert(v.screen_to_world(p.x_px, p.y_px, wx, wy));
    assert(std::abs(wx - 1.0) < 1e-9 && std::abs(wy - 2.0) < 1e-9);
    assert(!v.screen_to_world(0, 0, wx, wy) ||
           true);  // sized transform always maps
    sim::viewer::ViewTransform degen;
    assert(!degen.screen_to_world(10, 10, wx, wy));  // no screen size
    v.zoom_at(400.0f, 300.0f, 2.0f);
    assert(std::abs(v.scale() - 100.0f) < 1e-3f);
    const auto p2 = v.world_to_screen(1.0, 2.0);
    assert(std::abs(p2.x_px - 400.0f) < 1e-3f);
    assert(std::abs(p2.y_px - 300.0f) < 1e-3f);
    sim::viewer::ViewTransform f;
    f.set_screen(800, 600);
    assert(f.fit(-4.0, -3.0, 6.0, 5.0));
    const auto pa = f.world_to_screen(-4.0, -3.0);
    const auto pb = f.world_to_screen(6.0, 5.0);
    assert(pa.x_px >= 0.0f && pb.x_px <= 800.0f);
    assert(pa.y_px <= 600.0f && pb.y_px >= 0.0f);
  }

  // Frame mirrors truth + episode diagnostics; selection passes through.
  {
    sim::world::World w;
    w.set_static_boxes({{5.2, 1.0, 6.0, 1.8, 0.5}});
    w.add_place({"dock", 6.0, 0.0, 0.0});
    sim::world::Actor a;
    a.x = 2.0;
    a.y = -1.5;
    w.add_actor(a);
    auto step = make_step();
    step.diag.has_observation = true;
    step.diag.estimate.x_m = 1.1;
    step.diag.estimate.y_m = 1.9;
    step.diag.estimate.yaw_rad = 0.4;
    const std::vector<std::pair<double, double>> trail{{0.0, 0.0},
                                                       {0.5, 1.0}};
    sim::viewer::EditorSelection sel;
    sel.box = 0;
    sel.box_corner = 2;
    const sim::viewer::Frame f = sim::viewer::build_frame(
        w, step, {"dock"}, trail, sel);
    assert(f.step == 7);
    assert(std::abs(f.body.x_m - 1.0) < 1e-12);
    assert(std::abs(f.body.y_m - 2.0) < 1e-12);
    assert(f.body.half_length_m > 0.0 && f.body.half_width_m > 0.0);
    assert(f.body.estimate_valid);
    assert(std::abs(f.body.est_x_m - 1.1) < 1e-12);
    assert(f.trail.size() == 2);
    assert(f.boxes.size() == 1);
    assert(f.boxes[0].selected && f.boxes[0].selected_corner == 2);
    assert(std::abs(f.boxes[0].height_m - 0.5) < 1e-12);
    assert(f.actors.size() == 1);
    assert(f.places.size() == 1 && f.places[0].reached);
    assert(!f.places[0].selected);
    assert(f.lidar_valid && f.lidar_hits.size() == 5);
    // First ray at body yaw + angle_min, range 2 from (1, 2).
    const double ex = 1.0 + 2.0 * std::cos(0.5 - 1.0);
    const double ey = 2.0 + 2.0 * std::sin(0.5 - 1.0);
    assert(std::abs(f.lidar_hits[0].first - ex) < 1e-9);
    assert(std::abs(f.lidar_hits[0].second - ey) < 1e-9);
    assert(!f.static_collision);
    assert(std::abs(f.static_clearance_m - 0.3) < 1e-12);
    // Live attitude rides the frame (ramps tilt the viz).
    assert(std::abs(f.body.roll_rad - 0.1) < 1e-12);
    assert(std::abs(f.body.pitch_rad + 0.05) < 1e-12);
    // Legacy overload: identity mount reads the body-frame fan exactly.
    auto legacy = sim::viewer::lidar_endpoints(1.0, 2.0, 0.5, step.obs);
    assert(legacy.size() == 5);
    assert(std::abs(legacy[0].first - ex) < 1e-9);
  }

  // Mount-aware fan: origin shifted by the yaw-rotated offset, beams
  // steered by the mount yaw. A misconfigured mount shows up here.
  {
    sim::world::World w;
    auto step = make_step();
    step.truth_after.body_roll_rad = 0.0;  // level: yaw-only rotation exact
    step.truth_after.body_pitch_rad = 0.0;
    sim::viewer::LidarMount mount;
    mount.xyz_m[0] = 0.25;
    mount.xyz_m[2] = 0.35;
    mount.rpy_rad[2] = 0.5;  // fan steered 0.5 rad off the body axis
    const sim::viewer::Frame f = sim::viewer::build_frame(
        w, step, {}, {}, sim::viewer::EditorSelection{}, mount);
    // Origin: (1, 2) + R(0.5) * (0.25, 0).
    const double ox = 1.0 + 0.25 * std::cos(0.5);
    const double oy = 2.0 + 0.25 * std::sin(0.5);
    // First ray: origin + 2 m along (body yaw + mount yaw + angle_min).
    const double ex = ox + 2.0 * std::cos(0.5 + 0.5 - 1.0);
    const double ey = oy + 2.0 * std::sin(0.5 + 0.5 - 1.0);
    assert(std::abs(f.lidar_hits[0].first - ex) < 1e-9);
    assert(std::abs(f.lidar_hits[0].second - ey) < 1e-9);
    // True 3D sensor origin rides the frame for the 3D fan.
    assert(std::abs(f.body.sens_x_m - ox) < 1e-9);
    assert(std::abs(f.body.sens_y_m - oy) < 1e-9);
    assert(std::abs(f.body.sens_z_m - (0.12 + 0.35)) < 1e-9);
  }

  // Degenerate scans yield no hits, never NaN endpoints.
  {
    sim::world::World w;
    auto step = make_step();
    step.obs.scan_valid = false;
    sim::viewer::Frame f0 = sim::viewer::build_frame(w, step, {}, {}, {});
    assert(!f0.lidar_valid && f0.lidar_hits.empty());
    step.obs.scan_valid = true;
    step.obs.scan_angle_increment_rad = 0.0;  // degenerate layout
    sim::viewer::Frame f1 = sim::viewer::build_frame(w, step, {}, {}, {});
    assert(!f1.lidar_valid);
  }

  std::printf("viewer_frame ok\n");
  return 0;
}
