#include <algorithm>
#include <cassert>
#include <cstdio>
#include <string>

#include "sim/experiment/experience_runtime.hpp"
#include "sim/viewer/experience_manifest.hpp"

int main() {
 using namespace lost_signal;
 sim::viewer::ExperienceManifest experience;
 std::string error;
 assert(sim::viewer::load_experience_manifest(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/slam_loop.json",
 experience, &error));
 assert(experience.startup_mode ==
 sim::viewer::ExperienceStartupMode::kSlamOnly);

 sim::experiment::ExperienceRuntimeConfig config;
 config.robot_path = experience.robot_path;
 config.sim_path = experience.sim_path;
 config.map_path = experience.map_path;
 config.scene_path = experience.scene_path;
 config.asset_manifest_path =
 std::string(TEST_SOURCE_DIR) + "/assets/models/manifest.json";
 config.objectives = experience.objectives;
 config.startup_mode = experience.startup_mode;
 sim::experiment::ExperienceRuntime runtime;
 assert(sim::experiment::build_experience_runtime(config, runtime, &error));

 std::size_t maximum_keyframes = 0;
 std::size_t maximum_loops = 0;
 std::size_t maximum_tracks = 0;
 bool used_slam_only = false;
 double first_loop_truth_distance_m = -1.0;
 std::size_t completed_objectives_at_first_loop = 0;
 for (std::uint64_t step = 0;
 step < experience.time_limit_steps &&
 !runtime.session->objectives_done(); ++step) {
 runtime.session->step_once();
 const auto* recorded = runtime.session->last_eval_step();
 assert(recorded != nullptr);
 maximum_keyframes = std::max(
 maximum_keyframes, recorded->diag.slam_keyframes);
 maximum_loops = std::max(maximum_loops, recorded->diag.slam_loops_closed);
 if (first_loop_truth_distance_m < 0.0 &&
 recorded->diag.slam_loops_closed > 0) {
 first_loop_truth_distance_m = std::hypot(
 recorded->truth_after.body_x_m, recorded->truth_after.body_y_m);
 completed_objectives_at_first_loop =
 runtime.session->objective_results().size();
 }
 maximum_tracks = std::max(maximum_tracks, recorded->diag.slam_tracks);
 used_slam_only = used_slam_only ||
 recorded->diag.localization_action == "slam-only";
 // A sensor frame never carries the simulator's actor-free reference scan.
 assert(!recorded->obs.scan_static_valid);
 assert(recorded->obs.static_ranges_m.empty());
 }

 const auto score = runtime.session->score();
 if (!runtime.session->objectives_done() ||
 !runtime.session->objectives_passed() || score.collision_steps != 0 ||
 maximum_loops == 0 || first_loop_truth_distance_m < 0.0 ||
 completed_objectives_at_first_loop < 3) {
 const auto* last = runtime.session->last_eval_step();
 std::fprintf(stderr,
 "slam experience done=%d passed=%d steps=%zu collisions=%zu "
 "keyframes=%zu loops=%zu tracks=%zu first_loop_objectives=%zu "
 "first_loop_truth=%.3f "
 "truth=(%.3f,%.3f,%.3f) "
 "est=(%.3f,%.3f,%.3f sigma=%.3f/%.3f vel=%.3f/%.3f) "
 "loc=%s safety=%s decision=%s/%s cmd=%.9f/%.9f candidates=%zu "
 "goal=%d/%s objectives=%zu active=%s track=%s loop=%s\n",
 runtime.session->objectives_done(),
 runtime.session->objectives_passed(), score.steps,
 score.collision_steps, maximum_keyframes, maximum_loops, maximum_tracks,
 completed_objectives_at_first_loop,
 first_loop_truth_distance_m,
 last != nullptr ? last->truth_after.body_x_m : 0.0,
 last != nullptr ? last->truth_after.body_y_m : 0.0,
 last != nullptr ? last->truth_after.body_yaw_rad : 0.0,
 last != nullptr ? last->diag.estimate.x_m : 0.0,
 last != nullptr ? last->diag.estimate.y_m : 0.0,
 last != nullptr ? last->diag.estimate.yaw_rad : 0.0,
 last != nullptr ? last->diag.estimate.position_sigma_m : 0.0,
 last != nullptr ? last->diag.estimate.heading_sigma_rad : 0.0,
 last != nullptr ? last->diag.estimate.forward_mps : 0.0,
 last != nullptr ? last->diag.estimate.yaw_rate_radps : 0.0,
 last != nullptr ? last->diag.localization_action.c_str() : "--",
 last != nullptr ? last->diag.safety_reason.c_str() : "--",
 last != nullptr ? last->diag.evidence.constraint_code.c_str() : "--",
 last != nullptr ? last->diag.evidence.reason_code.c_str() : "--",
 last != nullptr ? last->cmd.forward_mps : 0.0,
 last != nullptr ? last->cmd.yaw_rate_radps : 0.0,
 last != nullptr ? last->diag.evidence.moving_candidate_count : 0,
 last != nullptr ? static_cast<int>(last->diag.goal.state) : -1,
 last != nullptr ? last->diag.goal.detail.c_str() : "--",
 runtime.session->objective_results().size(),
 runtime.session->objectives().active_place().c_str(),
 last != nullptr ? last->diag.track_diagnostic.c_str() : "--",
 last != nullptr ? last->diag.slam_loop_diagnostic.c_str() : "--");
 for (const auto& result : runtime.session->objective_results()) {
 std::fprintf(stderr,
 " objective %s robot=%d truth=%d miss=%.3f yaw=%.3f "
 "speed=%.3f/%.3f stable=%u failures=",
 result.place_id.c_str(), result.robot_done, result.truth_arrived,
 result.truth_miss_m, result.truth_yaw_error_rad,
 result.truth_linear_speed_mps, result.truth_yaw_rate_radps,
 result.truth_stable_steps);
 for (const auto& failure : result.failure_reasons) {
 std::fprintf(stderr, "%s,", failure.c_str());
 }
 std::fprintf(stderr, "\n");
 }
 std::size_t seen_loops = 0;
 std::size_t seen_tracks = 0;
 double minimum_grip = 1.0;
 for (const auto& step : runtime.session->episode().steps()) {
 minimum_grip = std::min(minimum_grip, step.diag.grip_factor);
 if (step.diag.slam_tracks > seen_tracks) {
 seen_tracks = step.diag.slam_tracks;
 if (seen_tracks % 10 == 0) {
 std::fprintf(stderr,
 " track %zu step=%llu correction=%.3f/%.3f error=%.3f/%.3f "
 "odom=%.3f/%.3f\n",
 seen_tracks, static_cast<unsigned long long>(step.step),
 step.diag.track_correction_m, step.diag.track_correction_rad,
 std::hypot(step.truth_after.body_x_m - step.diag.estimate.x_m,
 step.truth_after.body_y_m - step.diag.estimate.y_m),
 std::abs(std::remainder(step.truth_after.body_yaw_rad -
 step.diag.estimate.yaw_rad, 2.0 * 3.14159265358979323846)),
 std::hypot(step.truth_after.body_x_m - step.diag.odometry_estimate.x_m,
 step.truth_after.body_y_m - step.diag.odometry_estimate.y_m),
 std::abs(std::remainder(step.truth_after.body_yaw_rad -
 step.diag.odometry_estimate.yaw_rad,
 2.0 * 3.14159265358979323846)));
 }
 }
 if (step.diag.slam_loops_closed <= seen_loops) continue;
 seen_loops = step.diag.slam_loops_closed;
 std::fprintf(stderr,
 " loop %zu step=%llu edge=%llu->%llu correction=%.3f/%.3f "
 "truth=(%.3f,%.3f,%.3f) est=(%.3f,%.3f,%.3f) score=%.3f/%.3f "
 "overlap=%.3f margin=%.3f/%.3f\n",
 seen_loops, static_cast<unsigned long long>(step.step),
 static_cast<unsigned long long>(step.diag.slam_loop_from_id),
 static_cast<unsigned long long>(step.diag.slam_loop_to_id),
 step.diag.slam_loop_correction_m,
 step.diag.slam_loop_correction_rad,
 step.truth_after.body_x_m, step.truth_after.body_y_m,
 step.truth_after.body_yaw_rad, step.diag.estimate.x_m,
 step.diag.estimate.y_m, step.diag.estimate.yaw_rad,
 step.diag.slam_loop_forward_score,
 step.diag.slam_loop_reverse_score, step.diag.slam_loop_overlap,
 step.diag.slam_loop_forward_margin,
 step.diag.slam_loop_reverse_margin);
 }
 std::fprintf(stderr, " minimum grip %.3f\n", minimum_grip);
 const std::size_t tail_begin = score.steps > 250 ? score.steps - 250 : 0;
 for (std::size_t i = tail_begin;
 i < runtime.session->episode().steps().size(); i += 5) {
 const auto& step = runtime.session->episode().steps()[i];
 std::fprintf(stderr,
 " tail step=%llu est=(%.3f,%.3f,%.3f) vel=%.3f/%.3f "
 "cmd=%.3f/%.3f route=%zu/%zu err=%.3f gear=%d "
 "constraint=%s reason=%s goal=%d/%s\n",
 static_cast<unsigned long long>(step.step),
 step.diag.estimate.x_m, step.diag.estimate.y_m,
 step.diag.estimate.yaw_rad, step.diag.estimate.forward_mps,
 step.diag.estimate.yaw_rate_radps, step.cmd.forward_mps,
 step.cmd.yaw_rate_radps, step.diag.evidence.route_segment,
 step.diag.evidence.route_point_count, step.diag.evidence.route_error_m,
 step.diag.evidence.requested_gear,
 step.diag.evidence.constraint_code.c_str(),
 step.diag.evidence.reason_code.c_str(),
 static_cast<int>(step.diag.goal.state), step.diag.goal.detail.c_str());
 }
 }
 assert(used_slam_only);
 assert(maximum_keyframes >= 10);
 assert(maximum_loops >= 1);
 // The authored route reaches east, north, and west before it starts the
 // homeward leg. A closure before those three independent mission events is
 // a false historical association. Truth distance remains failure telemetry
 // only; neither this temporal witness nor SIM truth enters ROBOT.
 assert(first_loop_truth_distance_m >= 0.0);
 assert(completed_objectives_at_first_loop >= 3);
 assert(runtime.session->objectives_done());
 assert(runtime.session->objectives_passed());
 assert(score.collision_steps == 0);
 assert(score.max_pos_err_m < 1.0);
 return 0;
}
