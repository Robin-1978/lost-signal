#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/experience_manifest.hpp"

int main() {
 using namespace lost_signal;
 sim::viewer::ExperienceManifest experience;
 std::string error;
 assert(sim::viewer::load_experience_manifest(
 std::string(TEST_SOURCE_DIR) +
 "/configs/experiences/safety_unknown_obstacle.json",
 experience, &error));

 robot::platform::RobotConfig robot_config;
 assert(robot::platform::load_robot_config(
 experience.robot_path, robot_config, &error));
 sim::scenario::ScenarioSession session(
 robot::platform::description_from_config(robot_config),
 sim::eval::motor_from_config(robot_config));
 sim::eval::SimConfig sim_config;
 assert(sim::eval::load_sim_config(experience.sim_path, sim_config, &error));
 session.apply_sim_config(sim_config);

 sim::eval::WorldFileContent world;
 assert(sim::eval::load_world_file(experience.map_path, world, &error));
 bool stale_obstacle_found = false;
 for (const auto& box : world.boxes) {
 if (box.box_id == "unreported_crates") {
 stale_obstacle_found = true;
 assert(!box.prior_map_visible);
 }
 }
 assert(stale_obstacle_found);
 session.setup_world().set_static_boxes(std::move(world.boxes));
 session.setup_world().set_places(std::move(world.places));
 session.setup_world().set_materials(std::move(world.materials));
 session.setup_world().set_material_zones(std::move(world.zones));
 session.setup_world().set_terrain_patches(std::move(world.patches));
 session.setup_world().set_support_slabs(std::move(world.slabs));
 session.setup_world().set_environment(std::move(world.environment));
 assert(world.initial_pose.has_value());
 const auto& prior = *world.initial_pose;
 session.set_initial_pose(prior.x_m, prior.y_m, prior.yaw_rad,
 prior.position_sigma_m, prior.heading_sigma_rad);
 session.set_objectives(experience.objectives);
 session.fit_map_window_to_world();

 bool robot_observed_constraint = false;
 double minimum_robot_clearance = std::numeric_limits<double>::infinity();
 for (std::uint64_t step = 0;
 step < experience.time_limit_steps && !session.objectives_done(); ++step) {
 session.step_once();
 const auto evidence = session.last_evidence();
 if (std::isfinite(evidence.validator_static_clearance_m)) {
 minimum_robot_clearance = std::min(
 minimum_robot_clearance, evidence.validator_static_clearance_m);
 }
 robot_observed_constraint = robot_observed_constraint ||
 evidence.constraint_code == "static_obstacle" ||
 evidence.reason_code.find("proximity") != std::string::npos ||
 evidence.reason_code.find("blocked") != std::string::npos;
 }

 const auto score = session.score();
 const auto* last = session.last_eval_step();
 if (last != nullptr &&
 (!session.objectives_done() || !session.objectives_passed() ||
 score.collision_steps != 0 || score.min_clearance_m <
 robot_config.navigation.clearance_margin_m)) {
 std::fprintf(stderr,
 "safety_lab done=%d passed=%d truth=(%.3f,%.3f,%.3f) "
 "estimate=(%.3f,%.3f,%.3f) goal=%d/%s safety=%s "
 "decision=%s/%s cmd=(%.3f,%.3f) proposed=(%.3f,%.3f) "
 "target=(%.3f,%.3f) candidates=%zu brake=%d/%.3f "
 "est_vel=(%.3f,%.3f) "
 "truth_vel=(%.3f,%.3f) slip=%d/%d "
 "clear=%.3f collisions=%llu steps=%zu/%llu\n",
 session.objectives_done(), session.objectives_passed(),
 last->truth_after.body_x_m, last->truth_after.body_y_m,
 last->truth_after.body_yaw_rad, last->diag.estimate.x_m,
 last->diag.estimate.y_m, last->diag.estimate.yaw_rad,
 static_cast<int>(last->diag.goal.state), last->diag.goal.detail.c_str(),
 last->diag.safety_reason.c_str(),
 last->diag.evidence.constraint_code.c_str(),
 last->diag.evidence.reason_code.c_str(), last->cmd.forward_mps,
 last->cmd.yaw_rate_radps, last->diag.evidence.proposed_speed_mps,
 last->diag.evidence.proposed_yaw_rate_radps,
 last->diag.evidence.target_speed_mps,
 last->diag.evidence.target_yaw_rate_radps,
 last->diag.evidence.moving_candidate_count,
 last->diag.braking_active, last->diag.braking_distance_m,
 last->diag.estimate.forward_mps,
 last->diag.estimate.yaw_rate_radps, last->truth_after.forward_mps,
 last->truth_after.yaw_rate_radps,
 last->truth_after.drive_slipping[0],
 last->truth_after.drive_slipping[1], score.min_clearance_m,
 static_cast<unsigned long long>(score.collision_steps),
 session.episode().steps().size(),
 static_cast<unsigned long long>(experience.time_limit_steps));
 }
 assert(session.objectives_done());
 assert(session.objectives_passed());
 assert(score.collision_steps == 0);
 assert(robot_observed_constraint || minimum_robot_clearance < 1.0);
 assert(score.min_clearance_m >=
 robot_config.navigation.clearance_margin_m);
 return 0;
}
