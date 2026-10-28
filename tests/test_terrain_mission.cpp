// Terrain boundary missions (SIM-side HIL through the standard seams).
// Low-mu traverse: the grip governor tightens on measured slip, the
// mission still completes with zero judge steps. Ridge traverse: real
// contact geometry (not the gravity hack) climbs and descends with zero
// judge steps. Truth is asserted offline only; the ROBOT saw measurements.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/scenario/scenario_session.hpp"

using namespace lost_signal;

namespace {

robot::platform::RobotDescription seed_diff_desc() {
  const std::string dir = std::string(TEST_CONFIG_DIR) + "/robot";
  robot::platform::RobotConfig cfg;
  std::string err;
  assert(robot::platform::load_robot_config(dir + "/differential.json", cfg,
                                            &err));
  return robot::platform::description_from_config(cfg);
}

sim::world::MotorParams seed_diff_motor() {
  const std::string dir = std::string(TEST_CONFIG_DIR) + "/robot";
  robot::platform::RobotConfig cfg;
  std::string err;
  assert(robot::platform::load_robot_config(dir + "/differential.json", cfg,
                                            &err));
  return sim::eval::motor_from_config(cfg);
}

void drive_until_done(sim::scenario::ScenarioSession& session, int max_steps) {
  for (int i = 0; i < max_steps && !session.objectives_done(); ++i) {
    session.step_once();
  }
}

}  // namespace

int main() {
 // Icy climb: 0.14 up-ramp with a 0.22-mu belt over launch + climb, goal
  // past the ridge via a mid-ramp stop. Steady climbing at this grade
 // needs ~16 N/wheel plus rolling/support losses against ~26 N of ice
 // traction. The margin is thin enough that every launch/restart demand
 // spike saturates, and the
  // mid-ramp stop manufactures a restart on grade. Saturation still pulls
  // past the grade need, so the climb progresses WHILE slipping (force
  // margin, not speed — a steeper grade would beach force-limited, which
  // no governor can fix). The ROBOT gets no material notice; adaptation is
  // purely measured slip -> grip, ratcheted by stop-go (fast tighten, slow
  // relax).
  {
    sim::scenario::ScenarioSession session(seed_diff_desc(),
                                           seed_diff_motor());
 session.set_startup_mode(robot::autonomy::StartupMode::kSeedAndTrack);
 // Isolate terrain/traction from mapless observability. The surveyed static
 // geometry and this authored operator prior cross normal ROBOT deployment
 // seams; no runtime truth is read or fed back.
 session.set_initial_pose(0.0, 0.0, 0.0, 0.02, 0.01);
 session.setup_world().set_static_boxes({
 {-1.0, 3.0, 0.0, 4.0, 1.5},
 {4.0, -4.0, 5.0, -3.0, 1.5},
 {9.0, 3.0, 10.0, 4.0, 1.5},
 {8.0, 1.5, 8.8, 2.0, 1.5},
 {8.0, -2.0, 8.8, -1.5, 1.5},
 });
    session.setup_world().add_place({"mid", 4.0, 0.0, 0.0});
    session.setup_world().add_place({"past", 9.0, 0.0, 0.0});
    session.setup_world().set_terrain_patches(
        {{"up", 2.0, -3.0, 6.0, 3.0, 0.0, 0.14, 0.0, 0.0, 0.0},
         {"down", 6.0, -3.0, 10.0, 3.0, 0.56, -0.14, 0.0, 0.0, 0.0}});
    session.setup_world().set_materials(
 {{"ice", 0.22, 0.005, 0.006, 1.0, 0.3}});
    session.setup_world().set_material_zones(
        {{"belt", -2.0, -3.0, 6.5, 3.0, "ice"}});
 // Build the surveyed raster from the complete authored world plus its
 // recovery margin. The legacy fixed [-10, 10] harness window ends only
 // one metre past this goal; a physically valid downhill overshoot can put
 // the ROBOT estimate outside that raster, where global planning must fail
 // closed. Production scenario loading performs this same fit.
 session.fit_map_window_to_world();
    session.set_objectives({"mid", "past"}, 1.0);
    sim::eval::SimConfig sim;
    sim.lidar_composite = true;
    session.apply_sim_config(sim);
 drive_until_done(session, 2500);
    const auto score = sim::eval::score_episode(session.episode());
    double min_grip = 1.0, max_z = 0.0;
    bool slip_seen = false;
    for (const auto& step : session.episode().steps()) {
      min_grip = std::min(min_grip, step.diag.grip_factor);
      max_z = std::max(max_z, step.truth_after.body_z_m);
      for (bool s : step.truth_after.drive_slipping) slip_seen |= s;
    }
 std::printf("icy climb done=%d collisions=%zu min_grip=%.2f max_z=%.2f "
 "slip=%d\n",
                session.objectives_done(), score.collision_steps, min_grip,
 max_z, slip_seen);
 if (!session.objectives_done() && !session.episode().steps().empty()) {
 const auto& last = session.episode().steps().back();
 std::fprintf(stderr,
 "icy climb stalled: active=%s completed=%zu truth=(%.3f,%.3f,%.3f) "
 "estimate=(%.3f,%.3f,%.3f) sigma=%.3f safety=%s localization=%s "
 "cmd=(%.3f,%.3f) goal=%d/%s decision=%s/%s route=%zu/%zu "
 "err=%.3f candidates=%zu grip=%.3f tracks=%zu track=%s "
 "score=%.3f reverse=%.3f overlap=%.3f\n",
 session.objectives().active_place().c_str(),
 session.objectives().completed(), last.truth_after.body_x_m,
 last.truth_after.body_y_m, last.truth_after.body_yaw_rad,
 last.diag.estimate.x_m, last.diag.estimate.y_m,
 last.diag.estimate.yaw_rad, last.diag.estimate.position_sigma_m,
 last.diag.safety_reason.c_str(),
 last.diag.localization_action.c_str(), last.cmd.forward_mps,
 last.cmd.yaw_rate_radps, static_cast<int>(last.diag.goal.state),
 last.diag.goal.detail.c_str(),
 last.diag.evidence.constraint_code.c_str(),
 last.diag.evidence.reason_code.c_str(),
 last.diag.evidence.route_segment,
 last.diag.evidence.route_point_count,
 last.diag.evidence.route_error_m,
 last.diag.evidence.moving_candidate_count,
 last.diag.grip_factor, last.diag.slam_tracks,
 last.diag.track_diagnostic.c_str(), last.diag.slam_loop_forward_score,
 last.diag.slam_loop_reverse_score, last.diag.slam_loop_overlap);
 }
    assert(session.objectives_done());
    assert(score.collision_steps == 0);
    assert(min_grip < 0.95);  // governor tightened on measured slip
    assert(slip_seen);        // plant reported traction saturation offline
    assert(max_z > 0.2);      // physically climbed (offline truth)
  }

  // Ridge traverse on real contact geometry: 0.1 up-ramp, 0.1 down-ramp,
  // goal past the ridge. Pins climb + descent under the survey elevation
  // (grade cost, passable) with the composite front end watching the face.
  // Terrain-aware validation reads the ramp face as surveyed ground (not a
  // wall), so the approach drives instead of grinding. Discrimination
  // (gripped transients stay clean) is pinned in test_autonomy, not here.
  {
    sim::scenario::ScenarioSession session(seed_diff_desc(),
                                           seed_diff_motor());
 session.set_startup_mode(robot::autonomy::StartupMode::kSeedAndTrack);
 // Same surveyed-map startup and declared deployment prior as the icy run.
 session.set_initial_pose(0.0, 0.0, 0.0, 0.02, 0.01);
 session.setup_world().set_static_boxes({
 {-1.0, 3.0, 0.0, 4.0, 1.5},
 {4.0, -4.0, 5.0, -3.0, 1.5},
 {9.0, 3.0, 10.0, 4.0, 1.5},
 });
    session.setup_world().add_place({"past", 9.0, 0.0, 0.0});
    session.setup_world().set_terrain_patches(
        {{"up", 2.0, -3.0, 6.0, 3.0, 0.0, 0.1, 0.0, 0.0, 0.0},
         {"down", 6.0, -3.0, 10.0, 3.0, 0.4, -0.1, 0.0, 0.0, 0.0}});
 session.fit_map_window_to_world();
    session.set_objectives({"past"}, 1.0);
    sim::eval::SimConfig sim;
    sim.lidar_composite = true;
    session.apply_sim_config(sim);
    drive_until_done(session, 2500);
    const auto score = sim::eval::score_episode(session.episode());
    double max_z = 0.0, min_grip = 1.0;
    for (const auto& step : session.episode().steps()) {
      max_z = std::max(max_z, step.truth_after.body_z_m);
      min_grip = std::min(min_grip, step.diag.grip_factor);
    }
 std::printf("ridge done=%d collisions=%zu max_z=%.2f min_grip=%.2f\n",
                session.objectives_done(), score.collision_steps, max_z,
                min_grip);
 if (!session.objectives_done() && !session.episode().steps().empty()) {
 const auto& last = session.episode().steps().back();
 std::fprintf(stderr,
 "ridge stalled: active=%s completed=%zu truth=(%.3f,%.3f,%.3f) "
 "estimate=(%.3f,%.3f,%.3f) sigma=%.3f goal=%d/%s "
 "safety=%s decision=%s/%s cmd=(%.3f,%.3f) grip=%.3f\n",
 session.objectives().active_place().c_str(),
 session.objectives().completed(), last.truth_after.body_x_m,
 last.truth_after.body_y_m, last.truth_after.body_yaw_rad,
 last.diag.estimate.x_m, last.diag.estimate.y_m,
 last.diag.estimate.yaw_rad, last.diag.estimate.position_sigma_m,
 static_cast<int>(last.diag.goal.state), last.diag.goal.detail.c_str(),
 last.diag.safety_reason.c_str(),
 last.diag.evidence.constraint_code.c_str(),
 last.diag.evidence.reason_code.c_str(), last.cmd.forward_mps,
 last.cmd.yaw_rate_radps, last.diag.grip_factor);
 }
    assert(session.objectives_done());
    assert(score.collision_steps == 0);
    assert(max_z > 0.25);  // physically climbed (offline truth)
    // min_grip printed above for ops, not asserted (see note above).
  }

  std::printf("terrain_mission ok\n");
  return 0;
}
