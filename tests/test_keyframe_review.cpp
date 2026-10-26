#include <cassert>
#include <cmath>

#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/keyframe_review.hpp"

int main() {
 using namespace lost_signal;
 sim::eval::EpisodeRecorder episode;
 for (int i = 0; i < 9; ++i) {
 sim::eval::EvalStep step;
 step.step = static_cast<std::uint64_t>(i);
 step.diag.has_observation = i >= 1;
 step.diag.estimate.stamp_ns = i * 20'000'000;
 step.diag.estimate.forward_mps = i < 6 ? 0.4 : (i == 6 ? 0.04 : 0.0);
 step.diag.estimate.yaw_rate_radps = 0.0;
 step.cmd.forward_mps = i < 5 ? 0.4 : 0.0;
 step.obs_accepted = i >= 1;
 step.obs.scan_valid = i >= 1;
 step.obs.scan_header = {1, i * 20'000'000, "test.lidar",
 robot::core::Frame::kLidarLink};
 step.diag.evidence.action_code = i == 5 ? "stop" : "move";
 step.diag.evidence.reason_code = i == 5 ? "goal-settle" : "tracking";
 episode.record(std::move(step));
 }

 sim::viewer::KeyframeReviewConfig config;
 config.maximum_forward_speed_mps = 0.05;
 config.maximum_yaw_rate_radps = 0.05;
 config.settle_samples = 2;
 const auto keyframes = sim::viewer::extract_review_keyframes(episode, config);
 assert(keyframes.size() == 3);
 assert(keyframes[0].kind ==
 sim::viewer::ReviewKeyframeKind::kFirstObservation);
 assert(keyframes[0].episode_index == 1);
 assert(keyframes[0].evidence.find("test.lidar") != std::string::npos);
 assert(keyframes[1].kind ==
 sim::viewer::ReviewKeyframeKind::kBrakeCommand);
 assert(keyframes[1].episode_index == 5);
 assert(keyframes[1].evidence == "stop / goal-settle");
 assert(keyframes[2].kind ==
 sim::viewer::ReviewKeyframeKind::kStableStop);
 assert(keyframes[2].episode_index == 7);

 sim::eval::EpisodeRecorder empty;
 assert(sim::viewer::extract_review_keyframes(empty, config).empty());
 config.settle_samples = 0;
 assert(sim::viewer::extract_review_keyframes(episode, config).empty());

 // Selecting a review bookmark rebuilds every dynamic authority from one
 // episode index. Current World actors and future trail samples must not
 // leak into the historical frame.
 sim::world::World world;
 sim::world::Actor current_actor;
 current_actor.x = 99.0;
 world.add_actor(current_actor);
 sim::eval::EpisodeRecorder history;
 for (int i = 0; i < 3; ++i) {
 sim::eval::EvalStep step;
 step.step = static_cast<std::uint64_t>(100 + i);
 step.truth_after.step = static_cast<std::uint64_t>(200 + i);
 step.truth_after.body_x_m = 10.0 + i;
 step.truth_after.body_y_m = -2.0 - i;
 step.obs.stamp_ns = (i + 1) * 20'000'000;
 step.obs.sequence = static_cast<std::uint64_t>(10 + i);
 step.diag.has_observation = true;
 step.diag.estimate.valid = true;
 step.diag.estimate.stamp_ns = step.obs.stamp_ns;
 step.diag.estimate.x_m = 20.0 + i;
 step.diag.evidence.sequence = static_cast<std::uint64_t>(30 + i);
 step.cmd.forward_mps = 0.1 * i;
 step.presentation.captured = true;
 sim::world::WheelViz wheel;
 wheel.spin_rad = 0.5 + i;
 step.presentation.wheels.push_back(wheel);
 step.presentation.actors.push_back(
 {1.0 + i, 2.0 + i, 0.2, 0.0, 0.4, 1.7, true});
 history.record(std::move(step));
 }
 const auto historical = sim::viewer::build_episode_frame(
 world, history, 1, {}, {}, {}, 2);
 assert(historical.has_value());
 assert(historical->step == 101);
 assert(historical->body.x_m == 11.0);
 assert(historical->truth.x_m == 11.0);
 assert(std::abs(historical->observation.stamp_s - 0.04) < 1e-12);
 assert(historical->estimate.map_x_m == 21.0);
 assert(historical->decision.sequence == 31);
 assert(historical->decision.command_forward_mps == 0.1);
 assert(historical->body.wheels.size() == 1);
 assert(historical->body.wheels[0].spin_rad == 1.5);
 assert(historical->actors.size() == 1);
 assert(historical->actors[0].x_m == 2.0);
 assert(historical->actors[0].pedestrian);
 assert(historical->trail.size() == 2);
 assert(historical->trail.front().first == 10.0);
 assert(historical->trail.back().first == 11.0);
 assert(!sim::viewer::build_episode_frame(
 world, history, 3, {}).has_value());
 return 0;
}
