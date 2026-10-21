#include "robot/autonomy/mapping/slam_session.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim::autonomy {

namespace {

SlamSessionConfig bind_map_identity(SlamSessionConfig config,
 runtime::MapManager& maps) {
 // A rebuilt revision must preserve the identity of the active map. MapManager
 // intentionally rejects an identity change, so callers should not have to
 // copy this value into a separate SLAM configuration by hand.
 if (const auto current = maps.current_snapshot(); current &&
 !current->map_id.empty()) {
 config.frontend.map_id = current->map_id;
 config.rebuild.map_id = current->map_id;
 }
 return config;
}

// The detector's temporal filter is a cost prefilter. It must never discard a
// pair the loop-closure gate would accept, so it never runs below the gate's
// own temporal contract.
LoopClosureDetectorConfig detector_config(const SlamSessionConfig& config) {
 auto detector = config.loop_closure;
 detector.minimum_temporal_separation_s = std::max(
 detector.minimum_temporal_separation_s,
 config.session.loop_closure.minimum_temporal_separation_s);
 return detector;
}

} // namespace

SlamSession::SlamSession(SlamSessionConfig config, runtime::MapManager& maps)
 : config_(bind_map_identity(std::move(config), maps)),
 frontend_(config_.frontend),
 session_(config_.session),
 detector_(detector_config(config_)),
 rebuilder_(config_.rebuild),
 publisher_(config_.publication),
 maps_(maps) {}

void SlamSession::reset() {
 frontend_.reset();
 session_.reset();
}

SlamSessionUpdate SlamSession::update(const ObservationFrame& observations,
 const EstimatedState& prior, runtime::TimePoint sample_time) {
 const auto frontend = frontend_.update(observations, prior, sample_time);
 SlamSessionUpdate output;
 output.state = frontend.state;
 output.pose = frontend.pose;
 output.published_map = frontend.published_map;
 output.map_revision = frontend.map_revision;
 output.diagnostic = frontend.diagnostic;
 if (frontend.state != MappingState::Mapping || !observations.lidar)
 return output;
 output.keyframe_created = false;

 const double stamp_s = sample_time.seconds();
 const auto ingested = session_.ingest(
 {frontend.pose.position, frontend.pose.heading_rad},
 *observations.lidar, stamp_s, prior);
 output.keyframe_created = ingested.created_keyframe;
 if (!ingested.accepted) {
 output.diagnostic += "; keyframe rejected: " + ingested.diagnostic;
 return output;
 }
 if (!ingested.created_keyframe)
 return output;

 const auto candidate = detector_.detect(session_.keyframes());
 if (!candidate)
 return output;
 const auto closure = session_.add_loop_closure(candidate->from_keyframe,
 candidate->to_keyframe, candidate->evidence);
 if (!closure.accepted) {
 output.diagnostic += "; loop rejected: " + closure.diagnostic;
 return output;
 }
 output.loop_closure_accepted = true;
 const Pose2 previous_pose = session_.keyframes().back().graph.initial_pose;
 const auto published = publisher_.optimize_rebuild_publish(
 session_, rebuilder_, maps_, stamp_s);
 if (!published.success) {
 output.diagnostic += "; loop publication failed: " + published.diagnostic;
 return output;
 }
 const Pose2 corrected_pose = session_.keyframes().back().graph.initial_pose;
 // Adopt the rigid part of the graph correction in the frontend. The submap
 // is a rigid body: corrections between its keyframes are published through
 // the rebuilt map, while the frontend must live in the corrected frame.
 const Pose2 correction = compose_pose(corrected_pose, inverse_pose(previous_pose));
 if (std::isfinite(correction.position.x) &&
 std::isfinite(correction.position.y) &&
 std::isfinite(correction.heading_rad)) {
 frontend_.apply_map_correction(correction.position, correction.heading_rad);
 output.pose.position = compose_pose(correction,
 {frontend.pose.position, frontend.pose.heading_rad}).position;
 output.pose.heading_rad = wrap_angle(frontend.pose.heading_rad +
 correction.heading_rad);
 }
 output.published_map = maps_.current_snapshot();
 output.map_revision = published.revision;
 output.diagnostic += "; loop closure published revision " +
 std::to_string(published.revision);
 return output;
}

} // namespace sim::autonomy
