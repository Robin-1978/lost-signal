#include "sim/viewer/keyframe_review.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace lost_signal::sim::viewer {
namespace {

std::string observation_sources(const robot::iface::ObservationFrame& obs) {
 std::set<std::string> sources;
 const auto add = [&](const robot::iface::MeasurementHeader& header) {
 if (header.valid()) sources.insert(header.source);
 };
 if (obs.scan_valid) add(robot::iface::scan_measurement_header(obs));
 if (obs.odometry_valid || obs.has_joints()) {
 add(robot::iface::motion_measurement_header(obs));
 }
 if (obs.imu_valid) add(robot::iface::imu_measurement_header(obs));
 if (sources.empty() && !obs.source.empty()) sources.insert(obs.source);
 std::ostringstream joined;
 for (const auto& source : sources) {
 if (joined.tellp() > 0) joined << " + ";
 joined << source;
 }
 return joined.str();
}

robot::core::MonotonicNs evidence_stamp(const eval::EvalStep& step) {
 if (step.diag.estimate.stamp_ns != robot::core::kInvalidTime) {
 return step.diag.estimate.stamp_ns;
 }
 return step.obs.stamp_ns;
}

bool same_event_class(ReviewKeyframeKind lhs, ReviewKeyframeKind rhs) {
 const auto sim_verdict = [](ReviewKeyframeKind kind) {
 return kind == ReviewKeyframeKind::kSimAccepted ||
 kind == ReviewKeyframeKind::kSimRejected;
 };
 return lhs == rhs || (sim_verdict(lhs) && sim_verdict(rhs));
}

} // namespace

std::optional<ReviewKeyframeAlignment> align_review_keyframes_by_event(
 const std::vector<ReviewKeyframe>& baseline,
 const std::vector<ReviewKeyframe>& variant,
 ReviewKeyframeKind selected_variant_kind) {
 const auto variant_found = std::find_if(
 variant.begin(), variant.end(), [&](const ReviewKeyframe& keyframe) {
 return same_event_class(keyframe.kind, selected_variant_kind);
 });
 const auto baseline_found = std::find_if(
 baseline.begin(), baseline.end(), [&](const ReviewKeyframe& keyframe) {
 return same_event_class(keyframe.kind, selected_variant_kind);
 });
 if (variant_found == variant.end() || baseline_found == baseline.end()) {
 return std::nullopt;
 }
 return ReviewKeyframeAlignment{
 static_cast<std::size_t>(baseline_found - baseline.begin()),
 static_cast<std::size_t>(variant_found - variant.begin())};
}

std::vector<ReviewKeyframe> extract_review_keyframes(
 const eval::EpisodeRecorder& episode, KeyframeReviewConfig config,
 KeyframeReviewOutcome outcome) {
 std::vector<ReviewKeyframe> out;
 const auto& steps = episode.steps();
 if (steps.empty() || !std::isfinite(config.maximum_forward_speed_mps) ||
 config.maximum_forward_speed_mps < 0.0 ||
 !std::isfinite(config.maximum_yaw_rate_radps) ||
 config.maximum_yaw_rate_radps < 0.0 || config.settle_samples <= 0) {
 return out;
 }

 for (std::size_t i = 0; i < steps.size(); ++i) {
 const auto& step = steps[i];
 if (!step.obs_accepted || !step.diag.has_observation) continue;
 const std::string sources = observation_sources(step.obs);
 out.push_back({ReviewKeyframeKind::kFirstObservation, i, step.step,
 evidence_stamp(step), "FIRST VALID OBSERVATION",
 sources.empty() ? "ROBOT accepted a timestamped observation"
 : "ROBOT accepted " + sources});
 break;
 }

 for (std::size_t i = 0; i < steps.size(); ++i) {
 const auto& step = steps[i];
 if (std::abs(step.cmd.forward_mps) <= 1e-9 &&
 std::abs(step.cmd.lateral_mps) <= 1e-9 &&
 std::abs(step.cmd.yaw_rate_radps) <= 1e-9 &&
 std::abs(step.cmd.steer_angle_rad) <= 1e-9) {
 continue;
 }
 std::ostringstream evidence;
 evidence << "ROBOT commanded forward " << step.cmd.forward_mps
 << " m/s and yaw rate " << step.cmd.yaw_rate_radps << " rad/s";
 out.push_back({ReviewKeyframeKind::kFirstMotion, i, step.step,
 evidence_stamp(step), "FIRST ROBOT MOTION", evidence.str()});
 break;
 }

 const auto stopped = [&](const eval::EvalStep& step) {
 return step.diag.has_observation &&
 std::abs(step.diag.estimate.forward_mps) <=
 config.maximum_forward_speed_mps &&
 std::abs(step.diag.estimate.yaw_rate_radps) <=
 config.maximum_yaw_rate_radps;
 };
 std::size_t run_begin = steps.size();
 for (std::size_t i = steps.size(); i > 0; --i) {
 if (!stopped(steps[i - 1])) break;
 run_begin = i - 1;
 }
 const std::size_t settle = static_cast<std::size_t>(config.settle_samples);
 if (run_begin < steps.size() && steps.size() - run_begin >= settle) {
 const std::size_t stable_index = run_begin + settle - 1;
 std::size_t last_moving = run_begin;
 bool have_moving_command = false;
 for (std::size_t i = run_begin; i > 0; --i) {
 const std::size_t candidate = i - 1;
 if (std::abs(steps[candidate].cmd.forward_mps) >
 config.maximum_forward_speed_mps ||
 std::abs(steps[candidate].cmd.yaw_rate_radps) >
 config.maximum_yaw_rate_radps) {
 last_moving = candidate;
 have_moving_command = true;
 break;
 }
 }
 if (have_moving_command && last_moving + 1 < steps.size()) {
 const std::size_t brake_index = last_moving + 1;
 const auto& brake = steps[brake_index];
 std::string evidence = brake.diag.evidence.action_code;
 if (!brake.diag.evidence.reason_code.empty()) {
 if (!evidence.empty()) evidence += " / ";
 evidence += brake.diag.evidence.reason_code;
 }
 out.push_back({ReviewKeyframeKind::kBrakeCommand, brake_index,
 brake.step, evidence_stamp(brake), "BRAKING COMMAND", evidence});
 }
 const auto& stable = steps[stable_index];
 out.push_back({ReviewKeyframeKind::kStableStop, stable_index, stable.step,
 evidence_stamp(stable), "STABLE STOP CONFIRMED",
 "ROBOT estimate remained inside configured arrival speed limits"});
 }

 for (std::size_t i = 0; i < steps.size(); ++i) {
 const auto& step = steps[i];
 if (!step.diag.goal.handle.valid() ||
 step.diag.goal.state != robot::iface::GoalState::kDone) {
 continue;
 }
 std::string evidence = "ROBOT goal state became Done";
 if (!step.diag.goal.place_id.empty()) {
 evidence += " for " + step.diag.goal.place_id;
 }
 if (!step.diag.goal.detail.empty()) {
 evidence += " / " + step.diag.goal.detail;
 }
 out.push_back({ReviewKeyframeKind::kRobotDone, i, step.step,
 evidence_stamp(step), "ROBOT REPORTED DONE", std::move(evidence)});
 break;
 }

 for (std::size_t i = 0; i < steps.size(); ++i) {
 const auto& step = steps[i];
 if (!step.judge.static_collision && !step.judge.dynamic_collision &&
 !step.judge.terrain_collision) {
 continue;
 }
 std::ostringstream evidence;
 evidence << "SIM contact judge:";
 if (step.judge.static_collision) evidence << " static";
 if (step.judge.dynamic_collision) evidence << " dynamic";
 if (step.judge.terrain_collision) evidence << " terrain";
 out.push_back({ReviewKeyframeKind::kCollision, i, step.step,
 evidence_stamp(step), "COLLISION RECORDED", evidence.str()});
 break;
 }

 if (outcome.terminal) {
 const std::size_t terminal_index = steps.size() - 1;
 const auto& terminal = steps[terminal_index];
 if (outcome.timed_out) {
 out.push_back({ReviewKeyframeKind::kTimeout, terminal_index, terminal.step,
 evidence_stamp(terminal), "EXPERIMENT TIME LIMIT",
 "The authored experiment budget expired before mission completion"});
 }
 if (outcome.sim_accepted) {
 out.push_back({ReviewKeyframeKind::kSimAccepted, terminal_index,
 terminal.step, evidence_stamp(terminal), "SIM ACCEPTED",
 "Independent physical arrival, stable-stop, and collision gates passed"});
 } else {
 std::ostringstream evidence;
 for (const auto& reason : outcome.rejection_reasons) {
 if (reason.empty()) continue;
 if (evidence.tellp() > 0) evidence << " / ";
 evidence << reason;
 }
 out.push_back({ReviewKeyframeKind::kSimRejected, terminal_index,
 terminal.step, evidence_stamp(terminal), "SIM REJECTED",
 evidence.tellp() > 0 ? evidence.str()
 : "Independent physical acceptance did not pass"});
 }
 }

 std::stable_sort(out.begin(), out.end(),
 [](const ReviewKeyframe& lhs, const ReviewKeyframe& rhs) {
 return lhs.episode_index < rhs.episode_index;
 });
 return out;
}

} // namespace lost_signal::sim::viewer
