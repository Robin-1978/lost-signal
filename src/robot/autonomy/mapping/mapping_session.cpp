#include "robot/autonomy/mapping/mapping_session.hpp"

#include <cmath>
#include <limits>
#include <utility>

namespace sim::autonomy {
namespace {

bool finite_pose(const Pose2& pose) {
 return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
 std::isfinite(pose.heading_rad);
}

} // namespace

MappingSession::MappingSession(MappingSessionConfig config)
 : config_(std::move(config)), loop_gate_(config_.loop_closure) {}

void MappingSession::reset() {
 graph_.clear();
 keyframes_.clear();
}

bool MappingSession::valid_config(std::string* reason) const {
 if (!(config_.keyframes.translation_threshold_m > 0.0) ||
 !(config_.keyframes.heading_threshold_rad > 0.0) ||
 !(config_.keyframes.maximum_interval_s > 0.0) ||
 config_.keyframes.maximum_keyframes == 0) {
 *reason = "invalid keyframe trigger configuration";
 return false;
 }
 for (double value : config_.odometry_information) {
 if (!std::isfinite(value)) {
 *reason = "odometry information is not finite";
 return false;
 }
 }
 return true;
}

bool MappingSession::valid_scan(const Scan& scan, double stamp_s,
 std::string* reason) const {
 if (scan.n_rays <= 0 || scan.ranges.size() != static_cast<std::size_t>(scan.n_rays) ||
 scan.hit.size() != scan.ranges.size() || !(scan.max_range > 0.0) ||
 !std::isfinite(scan.header.stamp_s) || !std::isfinite(stamp_s) || stamp_s < 0.0 ||
 std::abs(scan.header.stamp_s - stamp_s) > 0.05) {
 *reason = "invalid or stale keyframe scan";
 return false;
 }
 for (const double range : scan.ranges) {
 if (!std::isfinite(range) || range < 0.0 || range > scan.max_range) {
 *reason = "keyframe scan contains an invalid range";
 return false;
 }
 }
 return true;
}

bool MappingSession::trigger(const Pose2& pose, double stamp_s) const {
 if (keyframes_.empty())
 return true;
 const auto& previous = keyframes_.back().graph;
 return pose_translation_error(previous.initial_pose, pose) >=
 config_.keyframes.translation_threshold_m ||
 std::abs(wrap_angle(previous.initial_pose.heading_rad - pose.heading_rad)) >=
 config_.keyframes.heading_threshold_rad ||
 stamp_s - previous.stamp_s >= config_.keyframes.maximum_interval_s;
}

MappingSessionUpdate MappingSession::ingest(Pose2 pose, Scan scan, double stamp_s) {
 MappingSessionUpdate output;
 std::string reason;
 if (!valid_config(&reason)) {
 output.diagnostic = std::move(reason);
 return output;
 }
 if (!finite_pose(pose) || !std::isfinite(stamp_s) || stamp_s < 0.0) {
 output.diagnostic = "invalid keyframe pose or timestamp";
 return output;
 }
 if (!valid_scan(scan, stamp_s, &reason)) {
 output.diagnostic = std::move(reason);
 return output;
 }
 if (!keyframes_.empty() && stamp_s <= keyframes_.back().graph.stamp_s) {
 output.diagnostic = "keyframe timestamp is not increasing";
 return output;
 }
 // A normal scan is accepted by the front end even when it does not carry
 // enough motion to become a retained keyframe.
 output.accepted = true;
 if (!trigger(pose, stamp_s)) {
 output.diagnostic = "scan retained by front end; keyframe threshold not reached";
 return output;
 }
 if (keyframes_.size() >= config_.keyframes.maximum_keyframes) {
 output.accepted = false;
 output.diagnostic = "mapping session keyframe budget exhausted";
 return output;
 }
 const std::uint64_t id = static_cast<std::uint64_t>(keyframes_.size()) + 1;
 PoseGraphKeyframe graph_keyframe;
 graph_keyframe.id = id;
 graph_keyframe.stamp_s = stamp_s;
 graph_keyframe.initial_pose = pose;
 graph_keyframe.fixed = keyframes_.empty();
 if (!graph_.add_keyframe(graph_keyframe, &reason)) {
 output.accepted = false;
 output.diagnostic = std::move(reason);
 return output;
 }
 if (keyframes_.size() >= 1) {
 const auto& previous = keyframes_.back().graph;
 PoseGraphConstraint constraint;
 constraint.id = id;
 constraint.from = previous.id;
 constraint.to = id;
 constraint.measured_relative_pose = relative_pose(previous.initial_pose, pose);
 constraint.information = config_.odometry_information;
 constraint.kind = PoseGraphConstraintKind::Odometry;
 if (!graph_.add_constraint(constraint, &reason)) {
 graph_.clear();
 // Rebuild is intentionally not attempted here: this is an invalid session
 // input and the caller must discard/replay the session deterministically.
 keyframes_.clear();
 output.accepted = false;
 output.diagnostic = "failed to add keyframe odometry constraint: " + reason;
 return output;
 }
 }
 keyframes_.push_back({graph_keyframe, std::move(scan)});
 output.created_keyframe = true;
 output.keyframe_id = id;
 output.diagnostic = "keyframe created";
 return output;
}

MappingSessionUpdate MappingSession::add_loop_closure(
 std::uint64_t from, std::uint64_t to, LoopClosureEvidence evidence) {
 MappingSessionUpdate output;
 const auto* from_keyframe = graph_.find_keyframe(from);
 const auto* to_keyframe = graph_.find_keyframe(to);
 if (!from_keyframe || !to_keyframe) {
 output.diagnostic = "loop closure references an unknown keyframe";
 return output;
 }
 const auto decision = loop_gate_.validate(*from_keyframe, *to_keyframe, evidence);
 if (!decision.accepted) {
 output.diagnostic = decision.diagnostic;
 return output;
 }
 PoseGraphConstraint constraint;
 constraint.id = (static_cast<std::uint64_t>(graph_.constraint_count()) + 1U) << 32U |
 (from ^ to);
 if (constraint.id == 0)
 constraint.id = static_cast<std::uint64_t>(graph_.constraint_count()) + 1U;
 constraint.from = from;
 constraint.to = to;
 constraint.measured_relative_pose = evidence.measured_relative_pose;
 constraint.information = evidence.information;
 constraint.kind = PoseGraphConstraintKind::LoopClosure;
 std::string reason;
 if (!graph_.add_constraint(constraint, &reason)) {
 output.diagnostic = "failed to add loop closure: " + reason;
 return output;
 }
 output.accepted = true;
 output.created_keyframe = false;
 output.keyframe_id = to;
 output.diagnostic = decision.diagnostic;
 return output;
}

} // namespace sim::autonomy
