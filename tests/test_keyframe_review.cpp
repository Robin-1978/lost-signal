#include <cassert>

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
 return 0;
}
