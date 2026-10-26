#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/actor_behavior.hpp"

namespace {

using namespace lost_signal;

robot::platform::RobotConfig ackermann_config() {
 robot::platform::RobotConfig config;
 std::string error;
 assert(robot::platform::load_robot_config(
 std::string(TEST_CONFIG_DIR) + "/robot/ackermann.json", config, &error));
 return config;
}

void run_phase(double actor_start_y_m, bool expect_predictive_response) {
 const robot::platform::RobotConfig config = ackermann_config();
 sim::scenario::ScenarioSession session(
 robot::platform::description_from_config(config),
 sim::eval::motor_from_config(config));

 sim::eval::SimConfig sim_config;
 sim_config.lidar_beams = 72;
 sim_config.lidar_max_range_m = 10.0f;
 session.apply_sim_config(sim_config);
 session.set_episode_history_enabled(false);

 // A surveyed wall behind the crossing has a robot-width opening. When the
 // pose covariance is broad, a vehicle in front of this wall is deliberately
 // not trusted from one map-relative scan; temporal motion evidence must
 // still survive and drive the ordinary predictor/validator/safety chain.
 session.setup_world().set_static_boxes({
 {5.0, -6.0, 5.3, -0.8, 1.5},
 {5.0, 0.8, 5.3, 6.0, 1.5},
 {9.5, -2.0, 9.8, 2.0, 1.5},
 });
 session.setup_world().add_place(
 {"east", 8.0, 0.0, 0.0});
 session.set_initial_pose(
 0.0, 0.0, 0.0, 0.35, 0.05);

 sim::world::Actor crosser;
 crosser.id = "anonymous-crossing-vehicle";
 crosser.x = 4.0;
 crosser.y = actor_start_y_m;
 crosser.radius_m = 0.7;
 crosser.height_m = 2.0;
 assert(sim::world::ActorBehaviorRegistry::global().create(
 "waypoints", crosser.behavior));
 crosser.behavior.waypoints = {{4.0, 6.0}};
 crosser.behavior.speed_mps = 1.4;
 crosser.behavior.loop = false;
 session.setup_world().add_actor(crosser);
 session.fit_map_window_to_world();
 session.set_objectives({"east"}, 0.5);

 bool saw_dynamic_track = false;
 bool saw_predictive_response = false;
 bool resumed_after_response = false;
 for (int step = 0; step < 2000 && !session.objectives_done(); ++step) {
 session.step_once();
 const auto* sample = session.last_eval_step();
 if (sample == nullptr) continue;
 saw_dynamic_track |= !sample->diag.dynamic_tracks.empty();
 const std::string& constraint = sample->diag.evidence.constraint_code;
 const std::string& reason = sample->diag.evidence.reason_code;
 const bool predictive_response =
 constraint == "dynamic_escape" ||
 constraint == "dynamic_clearance" ||
 reason == "route-conflict-hold" ||
 reason == "route-conflict-speed-cap" ||
 reason == "route-conflict-no-safe-hold";
 saw_predictive_response |= predictive_response;
 resumed_after_response |= saw_predictive_response &&
 !predictive_response && sample->cmd.forward_mps > 0.2;
 }

 const auto score = session.score();
 const auto* last = session.last_eval_step();
 const bool made_progress = last != nullptr &&
 last->truth_after.body_x_m > 3.5;
 if (score.collision_steps != 0 || !made_progress ||
 !saw_dynamic_track ||
 (expect_predictive_response &&
 (!saw_predictive_response || !resumed_after_response))) {
 std::fprintf(stderr,
 "crossing phase %.2f failed: done=%d collisions=%zu track=%d "
 "response=%d resumed=%d truth=(%.3f,%.3f) estimate=(%.3f,%.3f) "
 "safety=%s decision=%s/%s\n",
 actor_start_y_m, session.objectives_done(), score.collision_steps,
 saw_dynamic_track, saw_predictive_response, resumed_after_response,
 last != nullptr ? last->truth_after.body_x_m : 0.0,
 last != nullptr ? last->truth_after.body_y_m : 0.0,
 last != nullptr ? last->diag.estimate.x_m : 0.0,
 last != nullptr ? last->diag.estimate.y_m : 0.0,
 last != nullptr ? last->diag.safety_reason.c_str() : "none",
 last != nullptr ? last->diag.evidence.constraint_code.c_str() : "none",
 last != nullptr ? last->diag.evidence.reason_code.c_str() : "none");
 }
 assert(score.collision_steps == 0);
 assert(made_progress);
 assert(saw_dynamic_track);
 assert(!expect_predictive_response || saw_predictive_response);
 assert(!expect_predictive_response || resumed_after_response);
}

} // namespace

int main() {
 // Actor reaches the crossing before, with, and after the nominal ego ETA.
 // These are arrival phases of one generic geometry, not scene IDs or
 // scripted future knowledge exposed to ROBOT.
 run_phase(-2.0, false);
 run_phase(-5.5, true);
 run_phase(-8.0, true);
 std::printf("dynamic crossing mission ok\n");
 return 0;
}
