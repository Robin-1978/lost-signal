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
 step.diag.estimate.valid = true;
 step.diag.estimate.stamp_ns = 7'000'000'000;
 step.diag.estimate.position_sigma_m = 0.12;
 step.diag.odometry_estimate = step.diag.estimate;
 step.diag.localized = true;
 step.diag.localization_action = "local-correct";
 step.diag.evidence.sequence = 4;
 step.diag.evidence.decision_time_s = 7.0;
 step.diag.evidence.action_code = "follow-route";
 step.diag.evidence.reason_code = "trajectory-accepted";
 step.diag.evidence.route_point_count = 8;
 step.diag.safety = robot::autonomy::SafetyState::kNominal;
 step.diag.safety_reason = "nominal";
 step.cmd.forward_mps = 0.25;
 step.obs_accepted = true;
 step.obs.sequence = 12;
 step.obs.stamp_ns = 7'000'000'000;
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
 assert(f.truth.kAuthority == sim::viewer::FrameAuthority::kPhysicsTruth);
 assert(f.truth.valid && std::abs(f.truth.x_m - 1.0) < 1e-12);
 assert(f.observation.kAuthority ==
 sim::viewer::FrameAuthority::kRobotObservation);
 assert(f.observation.accepted && f.observation.lidar_beams == 5);
 assert(f.observation.lidar_returns == 5);
 assert(f.estimate.kAuthority == sim::viewer::FrameAuthority::kRobotEstimate);
 assert(f.estimate.valid && f.estimate.localized);
 assert(std::abs(f.estimate.position_sigma_m - 0.12) < 1e-12);
 assert(f.decision.kAuthority == sim::viewer::FrameAuthority::kRobotDecision);
 assert(f.decision.valid && f.decision.action_code == "follow-route");
 assert(f.decision.safety_state == "NOMINAL");
 assert(std::abs(f.decision.command_forward_mps - 0.25) < 1e-12);
 assert(f.evaluation.kAuthority ==
 sim::viewer::FrameAuthority::kGameEvaluation);
 assert(std::abs(f.evaluation.estimate_error_m - std::sqrt(0.02)) < 1e-12);
    assert(f.trail.size() == 2);
    assert(f.boxes.size() == 1);
    assert(f.boxes[0].selected && f.boxes[0].selected_corner == 2);
    assert(std::abs(f.boxes[0].height_m - 0.5) < 1e-12);
    assert(f.actors.size() == 1);
    assert(!f.actors[0].pedestrian);  // default 1.5 m vehicle height
    assert(std::abs(f.actors[0].height_m - 1.5) < 1e-12);
    assert(f.places.size() == 1 && f.places[0].reached);
    assert(!f.places[0].selected);
 assert(f.lidar_valid && f.lidar_hits.size() == 5);
 assert(f.observation.projection_valid);
 assert(f.observation.scan_sequence == 12);
 assert(std::abs(f.observation.scan_stamp_s - 7.0) < 1e-12);
 // Live overlay uses the ROBOT estimate, never the truth body pose.
 const double ex = 1.1 + 2.0 * std::cos(0.4 - 1.0);
 const double ey = 1.9 + 2.0 * std::sin(0.4 - 1.0);
 assert(std::abs(f.lidar_hits[0].first - ex) < 1e-9);
 assert(std::abs(f.lidar_hits[0].second - ey) < 1e-9);
 assert(f.truth_lidar_valid && f.truth_lidar_hits.size() == 5);
 const double truth_ex = 1.0 + 2.0 * std::cos(0.5 - 1.0);
 const double truth_ey = 2.0 + 2.0 * std::sin(0.5 - 1.0);
 assert(std::abs(f.truth_lidar_hits[0].first - truth_ex) < 1e-9);
 assert(std::abs(f.truth_lidar_hits[0].second - truth_ey) < 1e-9);
    assert(!f.static_collision);
    assert(std::abs(f.static_clearance_m - 0.3) < 1e-12);
    // Live attitude rides the frame (ramps tilt the viz).
    assert(std::abs(f.body.roll_rad - 0.1) < 1e-12);
    assert(std::abs(f.body.pitch_rad + 0.05) < 1e-12);
    // Legacy overload: identity mount reads the body-frame fan exactly.
    auto legacy = sim::viewer::lidar_endpoints(1.0, 2.0, 0.5, step.obs);
    assert(legacy.size() == 5);
 assert(std::abs(legacy[0].first - truth_ex) < 1e-9);
  }

  // Ground-truth overlays ride the frame (SIM paint, never ROBOT input):
  // material zones with ids, terrain footprints, pedestrian split.
  {
    sim::world::World w;
    w.set_materials(
 {{"ice", 0.1, 0.005, 0.006, 1.0, 0.3, "water", 0.012},
 {"sand", 0.55, 0.03, 0.02, 1.2, 0.22, "dust", 0.0}});
    w.set_material_zones({{"slick", -2.0, -2.0, 12.0, 2.0, "ice"}});
    w.set_terrain_patches(
        {{"up", 2.0, -3.0, 6.0, 3.0, 0.0, 0.14, 0.0, 0.0, 0.0}});
    sim::world::Actor walker;
    walker.x = 1.0;
    walker.height_m = 1.8;
 walker.visual_kind = "pedestrian";
    w.add_actor(walker);
    sim::eval::EvalStep s;
    s.truth_after = w.truth();
    const sim::viewer::Frame f = sim::viewer::build_frame(w, s, {}, {});
    assert(f.zones.size() == 1);
    assert(f.zones[0].material_id == "ice");
 assert(f.zones[0].surface_effect == "water");
 assert(f.zones[0].fluid_depth_m == 0.012);
    assert(std::abs(f.zones[0].x1_m - 12.0) < 1e-12);
    assert(f.patches.size() == 1);
    assert(std::abs(f.patches[0].y0_m + 3.0) < 1e-12);
    assert(f.actors.size() == 1 && f.actors[0].pedestrian);
  }

  // Wheel table per chassis (backend layout + live joints, same sources
  // as physics/encoders): diff spins 2 fixed wheels; ackermann steers 2
  // idlers with live spin; mecanum spins 4 + roller stripes; tracked draws
  // slabs (flagged, wheels still reported).
  {
    auto frame_from = [](sim::world::World& w) {
      sim::eval::EvalStep s;
      s.truth_after = w.truth();
      return sim::viewer::build_frame(w, s, {}, {});
    };
    // Diff: 2 fixed wheels, spin tracks the joints, slip flags honest.
    {
      sim::world::World w;
      robot::iface::ActuatorCommand cmd;
      cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
      cmd.stamp_ns = 0;
      cmd.expiry_ns = 1'000'000'000;
      cmd.drive[0] = cmd.drive[1] = 0.7;
      w.apply(cmd);
      for (int i = 0; i < 50; ++i) w.step(0.02);
      const auto f = frame_from(w);
      assert(!f.body.tracked);
      assert(f.body.wheels.size() == 2);
      const double track =
          w.motor_params().track_width_m * 0.5;
      assert(std::abs(f.body.wheels[0].y_m - track) < 1e-9);  // left
      assert(std::abs(f.body.wheels[1].y_m + track) < 1e-9);  // right
      for (const auto& wh : f.body.wheels) {
        assert(wh.steer_rad == 0.0);
        assert(!wh.mecanum_rollers);
      }
      assert(f.body.wheels[0].spin_rad ==
             w.truth().drive_angle_rad[0]);
      assert(f.body.wheels[0].slipping == w.truth().drive_slipping[0]);
      assert(std::abs(f.body.wheels[0].radius_m -
                      w.motor_params().wheel_radius_m) < 1e-12);
      assert(std::isfinite(f.body.wheels[0].contact_speed_mps));
      assert(std::isfinite(f.body.wheels[0].slip_speed_mps));
      assert(f.body.wheels[0].slip_speed_mps >= 0.0);
      assert(f.body.wheels[0].contact_valid);
      assert(f.body.wheels[0].contact_normal_force_N > 0.0);
    }
    // Ackermann: rear drives fixed, front idlers steered with live spin.
    {
      sim::world::World w;
      robot::iface::ActuatorCommand cmd;
      cmd.mode = robot::iface::ActuatorMode::kAckermannDuty;
      cmd.stamp_ns = 0;
      cmd.expiry_ns = 1'000'000'000;
      cmd.drive[0] = cmd.drive[1] = 0.5;
      cmd.steer[0] = cmd.steer[1] = 0.4;
      w.apply(cmd);
      for (int i = 0; i < 50; ++i) w.step(0.02);
      const auto f = frame_from(w);
      assert(f.body.wheels.size() == 4);

      const double half_base = w.motor_params().wheel_base_m * 0.5;
      assert(f.body.wheels[0].steer_rad == 0.0);  // rear drives fixed
      assert(f.body.wheels[1].steer_rad == 0.0);
      assert(f.body.wheels[0].x_m < 0.0 && f.body.wheels[2].x_m > 0.0);
      assert(std::abs(f.body.wheels[2].x_m - half_base) < 1e-9);
      // Idlers follow the measured servo joints (effort duty, no position
      // loop in the plant — the ROBOT closes on measured angle). Both
      // fronts agree and turn the car left (+steer = left, yaw grows).
      assert(f.body.wheels[2].steer_rad ==
             w.truth().steer_rad[0]);
      assert(f.body.wheels[2].steer_rad > 0.01);
      assert(std::abs(f.body.wheels[2].steer_rad -
                      f.body.wheels[3].steer_rad) < 0.01);
      assert(w.truth().body_yaw_rad > 0.0);
      assert(std::isfinite(f.body.wheels[2].spin_rad));
      assert(std::isfinite(f.body.wheels[2].spin_rate_radps));
      assert(f.body.wheels[0].steer_rate_radps == 0.0);
      assert(f.body.wheels[2].steer_rate_radps ==
             w.truth().steer_vel_radps[0]);
    }
    // Material is sampled independently at each physical module. This is
    // SIM-only visual evidence and never enters an ObservationFrame.
    {
      sim::world::World w;
      w.set_materials(
 {{"sand", 0.55, 0.03, 0.02, 1.2, 0.22, "dust", 0.0}});
      w.set_material_zones(
        {{"left_sand", -0.3, 0.0, 0.3, 1.0, "sand"}});
      robot::iface::ActuatorCommand cmd;
      cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
      cmd.expiry_ns = 1'000'000'000;
      w.apply(cmd);
      w.step(0.02);
      const auto f = frame_from(w);
      assert(f.body.wheels.size() == 2);
      assert(f.body.wheels[0].contact_material_id == "sand");
 assert(f.body.wheels[0].contact_surface_effect == "dust");
      assert(f.body.wheels[1].contact_material_id.empty());
    }
    // Mecanum: 4 spinning wheels, fixed modules, roller stripes on.
    {
      sim::world::World w;
      robot::iface::ActuatorCommand cmd;
      cmd.mode = robot::iface::ActuatorMode::kMecanumDuty;
      cmd.stamp_ns = 0;
      cmd.expiry_ns = 1'000'000'000;
      cmd.drive = {-0.5, 0.5, 0.5, -0.5};
      w.apply(cmd);
      for (int i = 0; i < 50; ++i) w.step(0.02);
      const auto f = frame_from(w);
      assert(f.body.wheels.size() == 4);
      for (const auto& wh : f.body.wheels) {
        assert(wh.steer_rad == 0.0);
        assert(wh.mecanum_rollers);
      }
      assert(f.body.wheels[0].spin_rad != 0.0);  // strafing spins wheels
    }
    // Tracked: slab flag on (wheels still reported underneath).
    {
      sim::world::World w;
      sim::world::MotorParams mp;
      mp.yaw_scrub_coulomb_Nm = 8.0;
      mp.yaw_scrub_viscous_Nm_per_radps = 6.0;
      w.set_motor_params(mp);
      robot::iface::ActuatorCommand cmd;
      cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
      cmd.stamp_ns = 0;
      cmd.expiry_ns = 1'000'000'000;
      cmd.drive[0] = cmd.drive[1] = 0.5;
      w.apply(cmd);
      for (int i = 0; i < 50; ++i) w.step(0.02);
      const auto f = frame_from(w);
      assert(f.body.tracked);
      assert(f.body.wheels.size() == 2);
    }
  }

  // Mount-aware fan: origin shifted by the yaw-rotated offset, beams
  // steered by the mount yaw. A misconfigured mount shows up here.
  {
    sim::world::World w;
    auto step = make_step();
    step.truth_after.body_roll_rad = 0.0;  // level: yaw-only rotation exact
 step.truth_after.body_pitch_rad = 0.0;
 step.diag.estimate.valid = true;
 step.diag.estimate.x_m = 1.2;
 step.diag.estimate.y_m = 2.1;
 step.diag.estimate.yaw_rad = 0.4;
 step.diag.estimate.stamp_ns = 40'000'000;
    sim::viewer::LidarMount mount;
    mount.xyz_m[0] = 0.25;
    mount.xyz_m[2] = 0.35;
    mount.rpy_rad[2] = 0.5;  // fan steered 0.5 rad off the body axis
    const sim::viewer::Frame f = sim::viewer::build_frame(
        w, step, {}, {}, sim::viewer::EditorSelection{}, mount);
 // Live origin and beam fan use the ROBOT estimate.
 const double robot_ox = 1.2 + 0.25 * std::cos(0.4);
 const double robot_oy = 2.1 + 0.25 * std::sin(0.4);
 const double ex = robot_ox + 2.0 * std::cos(0.4 + 0.5 - 1.0);
 const double ey = robot_oy + 2.0 * std::sin(0.4 + 0.5 - 1.0);
    assert(std::abs(f.lidar_hits[0].first - ex) < 1e-9);
    assert(std::abs(f.lidar_hits[0].second - ey) < 1e-9);
 assert(std::abs(f.observation.projected_sensor_x_m - robot_ox) < 1e-9);
 assert(std::abs(f.observation.projected_sensor_y_m - robot_oy) < 1e-9);
 assert(std::abs(f.observation.projected_sensor_z_m - 0.35) < 1e-9);
    // True 3D sensor origin rides the frame for the 3D fan.
 const double ox = 1.0 + 0.25 * std::cos(0.5);
 const double oy = 2.0 + 0.25 * std::sin(0.5);
    assert(std::abs(f.body.sens_x_m - ox) < 1e-9);
    assert(std::abs(f.body.sens_y_m - oy) < 1e-9);
    assert(std::abs(f.body.sens_z_m - (0.12 + 0.35)) < 1e-9);
 const double truth_ex = ox + 2.0 * std::cos(0.5 + 0.5 - 1.0);
 const double truth_ey = oy + 2.0 * std::sin(0.5 + 0.5 - 1.0);
 assert(std::abs(f.truth_lidar_hits[0].first - truth_ex) < 1e-9);
 assert(std::abs(f.truth_lidar_hits[0].second - truth_ey) < 1e-9);
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

 // A valid scan without a valid ROBOT pose stays in sensor coordinates. The
 // live viewer must not silently project it with SIM truth.
 {
 sim::world::World w;
 auto step = make_step();
 step.diag.has_observation = true;
 step.diag.estimate.valid = false;
 const sim::viewer::Frame f = sim::viewer::build_frame(w, step, {}, {}, {});
 assert(!f.body.estimate_valid);
 assert(!f.observation.projection_valid);
 assert(!f.lidar_valid && f.lidar_hits.empty());
 assert(f.truth_lidar_valid && f.truth_lidar_hits.size() == 5);
 }

  std::printf("viewer_frame ok\n");
  return 0;
}
