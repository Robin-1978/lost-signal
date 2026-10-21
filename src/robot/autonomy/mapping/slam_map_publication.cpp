#include "robot/autonomy/mapping/slam_map_publication.hpp"

#include <cmath>
#include <cstddef>
#include <utility>

namespace sim::autonomy {

SlamMapPublication::SlamMapPublication(SlamMapPublicationConfig config)
 : config_(std::move(config)) {}

SlamMapPublicationResult SlamMapPublication::optimize_rebuild_publish(
 MappingSession& session, const KeyframeMapRebuilder& rebuilder,
 runtime::MapManager& map_manager, double publication_time_s
 ) const {
 SlamMapPublicationResult result;
 if (!std::isfinite(publication_time_s) || publication_time_s < 0.0) {
 result.diagnostic = "invalid map publication timestamp";
 return result;
 }
 const auto current = map_manager.current_snapshot();
 if (map_manager.state() != runtime::LifecycleState::Active || !current) {
 result.diagnostic = "map manager is not active";
 return result;
 }
 if (session.keyframes().empty()) {
 result.diagnostic = "mapping session has no keyframes";
 return result;
 }
 auto optimized_graph = session.graph();
 const auto optimized = optimized_graph.optimize(config_.optimization);
 if (!optimized.success) {
 result.diagnostic = "pose graph optimization failed: " + optimized.diagnostic;
 return result;
 }
 if (!(optimized.final_cost <= optimized.initial_cost + 1e-9) ||
 optimized.maximum_translation_correction_m >
 config_.maximum_translation_correction_m ||
 optimized.maximum_heading_correction_rad >
 config_.maximum_heading_correction_rad) {
 result.diagnostic = "pose graph correction exceeds publication gate";
 return result;
 }
 result.graph_optimized = true;
 const auto revision = current->revision + 1U;
 auto corrected_keyframes = session.keyframes();
 for (std::size_t index = 0; index < corrected_keyframes.size(); ++index)
 corrected_keyframes[index].graph.initial_pose =
 optimized_graph.keyframes()[index].initial_pose;
 const auto rebuilt = rebuilder.rebuild(corrected_keyframes, revision,
 publication_time_s);
 if (!rebuilt.success || !rebuilt.snapshot) {
 result.diagnostic = "map reconstruction failed: " + rebuilt.diagnostic;
 return result;
 }
 std::string reason;
 const auto map_result = map_manager.replace(*rebuilt.snapshot, &reason);
 result.map_result = map_result;
 result.revision = revision;
 if (map_result != runtime::MapUpdateResult::Accepted) {
 result.diagnostic = "map publication rejected: " + reason;
 return result;
 }
 // Keep retained session data aligned with the graph only after the immutable
 // map has been accepted. If this fails, the published revision remains valid
 // and the caller can discard/replay the session instead of mutating readers.
 if (!session.apply_optimized_poses(optimized_graph, &reason)) {
 result.diagnostic = "published map but failed to update session poses: " + reason;
 return result;
 }
 result.success = true;
 result.diagnostic = "optimized keyframe map published";
 return result;
}

} // namespace sim::autonomy
