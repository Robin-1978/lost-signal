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

} // namespace

std::vector<ReviewKeyframe> extract_review_keyframes(
 const eval::EpisodeRecorder& episode, KeyframeReviewConfig config) {
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
 return out;
}

} // namespace lost_signal::sim::viewer
