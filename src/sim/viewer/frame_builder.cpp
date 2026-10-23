#include "sim/viewer/frame_builder.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/transform.hpp"

namespace lost_signal::sim::viewer {

std::vector<std::pair<double, double>> lidar_endpoints(
    double body_x_m, double body_y_m, double body_yaw_rad,
    const robot::iface::ObservationFrame& obs, const LidarMount& mount) {
  std::vector<std::pair<double, double>> hits;
  if (!obs.scan_valid || obs.ranges_m.empty()) return hits;
  const double inc = obs.scan_angle_increment_rad;
  if (!(inc > 0.0) || !std::isfinite(inc)) return hits;
  // True sensor origin: mount offset rotated by body yaw (+ mount yaw on
  // the beam fan, same composition as generation).
  const double cy = std::cos(body_yaw_rad), sy = std::sin(body_yaw_rad);
  const double ox = body_x_m + mount.xyz_m[0] * cy - mount.xyz_m[1] * sy;
  const double oy = body_y_m + mount.xyz_m[0] * sy + mount.xyz_m[1] * cy;
  const double fan =
      (std::isfinite(mount.rpy_rad[2]) ? mount.rpy_rad[2] : 0.0);
  hits.reserve(obs.ranges_m.size());
  for (std::size_t i = 0; i < obs.ranges_m.size(); ++i) {
    const float r = obs.ranges_m[i];
    if (!std::isfinite(r) || r <= 0.0f) continue;
    const double a =
        body_yaw_rad + fan + obs.scan_angle_min_rad + i * inc;
    hits.emplace_back(ox + r * std::cos(a), oy + r * std::sin(a));
  }
  return hits;
}

Frame build_frame(const world::World& world, const eval::EvalStep& step,
                  const std::vector<std::string>& reached_ids,
                  const std::vector<std::pair<double, double>>& trail,
                  const EditorSelection& sel, const LidarMount& mount) {
  Frame f;
  f.step = step.step;
  const world::WorldTruth& t = step.truth_after;
  f.body.x_m = t.body_x_m;
  f.body.y_m = t.body_y_m;
  f.body.yaw_rad = t.body_yaw_rad;
  f.body.z_m = t.body_z_m;
  f.body.roll_rad = t.body_roll_rad;
  f.body.pitch_rad = t.body_pitch_rad;
  // Wheels from the backend wheel table (layout + live joints + slip
  // flags — the same sources physics and the encoder provider read).
  // Layouts: (2,0)+scrub = skid-steer slabs, (2,0) diff, (2,2) ackermann
  // (rears driven, fronts steered idlers with live spin), (4,4) swerve,
  // (4,0) mecanum (+ static 45-degree roller stripes: rollers are
  // analytic-wrench state, never meshed, never drawn live).
  {
    const int nd = static_cast<int>(t.drive_count);
    const int ns = static_cast<int>(t.steer_count);
    const double scrub = world.motor_params().yaw_scrub_coulomb_Nm;
    f.body.tracked = (nd == 2 && ns == 0 && scrub > 0.0);
    const bool mecanum = (nd == 4 && ns == 0);
    world::WheelViz table[6];
    const int nw = world.wheels(table, 6);
    for (int i = 0; i < nw; ++i) {
      FrameWheel w;
      w.x_m = table[i].x_m;
      w.y_m = table[i].y_m;
      w.steer_rad = table[i].steer_rad;
      w.spin_rad = table[i].spin_rad;
      w.radius_m = world.motor_params().wheel_radius_m;
      w.slipping = table[i].slipping;
      w.mecanum_rollers = mecanum;
      f.body.wheels.push_back(w);
    }
  }
  // True 3D sensor origin (body + attitude composed with the mount — the
  // same composition generation runs, so the viz fan starts where the
  // beams do).
  {
    const robot::core::Quaternion bq = robot::core::quaternion_from_rpy(
        t.body_roll_rad, t.body_pitch_rad, t.body_yaw_rad);
    const robot::core::Vec3 off = robot::core::rotate_vector(
        bq, {mount.xyz_m[0], mount.xyz_m[1], mount.xyz_m[2]});
    f.body.sens_x_m = t.body_x_m + off.x;
    f.body.sens_y_m = t.body_y_m + off.y;
    f.body.sens_z_m = t.body_z_m + off.z;
  }
  const double base = world.motor_params().wheel_base_m;
  const double track = world.motor_params().track_width_m;
  if (base > 0.0) f.body.half_length_m = base * 0.5;
  if (track > 0.0) f.body.half_width_m = track * 0.5;
  if (step.diag.has_observation) {
    f.body.estimate_valid = true;
    f.body.est_x_m = step.diag.estimate.x_m;
    f.body.est_y_m = step.diag.estimate.y_m;
    f.body.est_yaw_rad = step.diag.estimate.yaw_rad;
  }
  f.trail = trail;
  const auto& boxes = world.static_boxes();
  f.boxes.reserve(boxes.size());
  for (std::size_t i = 0; i < boxes.size(); ++i) {
    FrameBox b;
    b.x0_m = std::min(boxes[i].x0, boxes[i].x1);
    b.y0_m = std::min(boxes[i].y0, boxes[i].y1);
    b.x1_m = std::max(boxes[i].x0, boxes[i].x1);
    b.y1_m = std::max(boxes[i].y0, boxes[i].y1);
    b.height_m = boxes[i].height_m;
    b.selected = (sel.box == static_cast<int>(i));
    b.selected_corner = b.selected ? sel.box_corner : -1;
    f.boxes.push_back(b);
  }
  for (const auto& a : world.actors()) {
    FrameActor fa;
    fa.x_m = a.x;
    fa.y_m = a.y;
    fa.radius_m = a.radius_m;
    fa.vx_mps = a.vx_mps;
    fa.vy_mps = a.vy_mps;
    fa.height_m = a.height_m;
    // No kind field on the SIM actor: the map converter writes 1.8 m for
    // pedestrians, 1.5 m for vehicles — the split reads that convention.
    fa.pedestrian = a.height_m > 1.6;
    f.actors.push_back(fa);
  }
  for (const auto& z : world.material_zones()) {
    FrameZone fz;
    fz.x0_m = std::min(z.x0, z.x1);
    fz.y0_m = std::min(z.y0, z.y1);
    fz.x1_m = std::max(z.x0, z.x1);
    fz.y1_m = std::max(z.y0, z.y1);
    fz.material_id = z.material_id;
    f.zones.push_back(fz);
  }
  for (const auto& p : world.terrain_patches()) {
    FramePatch fp;
    fp.x0_m = std::min(p.x0, p.x1);
    fp.y0_m = std::min(p.y0, p.y1);
    fp.x1_m = std::max(p.x0, p.x1);
    fp.y1_m = std::max(p.y0, p.y1);
    f.patches.push_back(fp);
  }
  const auto& places = world.places();
  f.places.reserve(places.size());
  for (std::size_t i = 0; i < places.size(); ++i) {
    FramePlace p;
    p.place_id = places[i].place_id;
    p.x_m = places[i].x_m;
    p.y_m = places[i].y_m;
    p.yaw_rad = places[i].yaw_rad;
    p.reached = std::find(reached_ids.begin(), reached_ids.end(),
                           places[i].place_id) != reached_ids.end();
    p.selected = (sel.place == static_cast<int>(i));
    f.places.push_back(p);
  }
  f.lidar_hits = lidar_endpoints(t.body_x_m, t.body_y_m, t.body_yaw_rad,
                                   step.obs, mount);
  f.lidar_valid = !f.lidar_hits.empty();
  f.static_collision = step.judge.static_collision;
  f.dynamic_collision = step.judge.dynamic_collision;
  f.static_clearance_m = step.judge.static_clearance_m;
  return f;
}

}  // namespace lost_signal::sim::viewer
